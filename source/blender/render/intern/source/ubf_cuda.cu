/*
 * ubf_cuda.cu — CUDA-бэкенд UBF. Этап A1: марш экранных отражений в BI на GPU.
 *
 * ПОЧЕМУ ИМЕННО ТАК
 *   CPU-версия марша (bi_ssr_march_probe + цикл в RE_fast_frame_resolve) —
 *   ЭТАЛОН. Ядро повторяет её операция за операцией, включая порядок слагаемых
 *   в скалярном произведении и приведение (int) с усечением к нулю. Чтобы
 *   совпадение было битовым, а не «похожим»:
 *     * компилируем с -fmad=false (иначе nvcc склеит mul+add и результат
 *       разойдётся в младших битах, а на длинном марше это переключит пиксель);
 *     * таблица времён шага t считается на CPU (там же, где powf) и приходит
 *       готовым массивом — в ядре нет ни powf, ни других трансцендентных;
 *     * деления и сравнения — обычные IEEE (fast math выключен).
 *   Поэтому проверка приёмки — не «картинка похожа», а max=0 на cmp.py.
 *
 * ЧТО ДЕЛАЕТ ЯДРО
 *   По каждому пикселю: если i = mir[4*idx] > 0 — это зеркальный пиксель.
 *   Считаем направление отражения D = V - 2(V·N)N, шагаем по таблице ts,
 *   на каждом шаге проецируем точку в кадр и сравниваем её глубину с глубиной
 *   буфера; первое попадание уточняем бисекцией (если bisect > 0). Наружу —
 *   индекс пикселя-попадания и (в режиме defer) маска промахов.
 *
 * ПАМЯТЬ
 *   Буферы устройства держим между кадрами (перевыделяем только при смене
 *   размера кадра). Цена копий G-буфера — это то, что надо ИЗМЕРИТЬ, а не
 *   предположить, поэтому печатаем kernel/копии/итог отдельно.
 */

#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ubf_cuda.h"

/* ------------------------------------------------------------------ ядро --- */

/* Одна проба марша: точка на луче в момент t. Индекс пикселя-попадания или -1.
 * Точная копия bi_ssr_march_probe() из RE_Rasterizer.c. */
__device__ int ubf_probe(const float *P0, const float *D,
                         float winx, float winy, const float *winmat,
                         const float *depth, int w, int h,
                         float t, float th)
{
	float P[3], wcl, sx, sy, zd, diff;
	int px, py, j;

	P[0] = P0[0] + t * D[0];
	P[1] = P0[1] + t * D[1];
	P[2] = P0[2] + t * D[2];
	if (P[2] >= -0.001f) return -1;          /* ушли за камеру */

	wcl = -P[2];
	sx = 0.5f * winx * ((winmat[0] * P[0] + winmat[12]) / wcl + 1.0f);
	sy = 0.5f * winy * ((winmat[5] * P[1] + winmat[13]) / wcl + 1.0f);

	px = (int)(sx * ((float)w / (float)winx));
	py = (int)(sy * ((float)h / (float)winy));
	if (px < 0 || py < 0 || px >= w || py >= h) return -1;

	j = py * w + px;
	zd = depth[j];
	if (zd >= 1e29f) return -1;              /* небо/пусто — не поверхность */

	diff = wcl - zd;
	if (th > 0.0f) {
		if (diff > 0.0f && diff <= th) return j;
	}
	else if (diff > zd * 0.01f + 0.01f) {
		return j;
	}
	return -1;
}

__global__ void ubf_ssr_march_kernel(const float *pos, const float *nrm,
                                     const float *view, const float *mir,
                                     const float *depth, float winx, float winy,
                                     const float *winmat, const float *ts,
                                     int w, int h, int steps, int bisect,
                                     int thick, int defer,
                                     int *hit_out, unsigned char *miss_out)
{
	const int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const float *P0, *N, *V;
	float D[3], dd, tprev, tnear = 0.0f, tfar = 0.0f, thfar = 0.0f;
	int k, hit = -1;

	if (idx >= w * h) return;
	if (mir[4 * idx] <= 0.0f) return;        /* не зеркальный пиксель */

	P0 = pos + 3 * idx;
	N  = nrm + 3 * idx;
	V  = view + 3 * idx;

	/* D = V - 2(V·N)N — порядок слагаемых как в CPU-эталоне. */
	dd = 2.0f * (V[0] * N[0] + V[1] * N[1] + V[2] * N[2]);
	D[0] = V[0] - dd * N[0];
	D[1] = V[1] - dd * N[1];
	D[2] = V[2] - dd * N[2];

	if (D[2] >= 0.0f) {                      /* луч уходит за камеру */
		if (defer) miss_out[idx] = 1;
		return;
	}

	tprev = 0.0f;
	for (k = 0; k < steps; k++) {
		const float t = ts[k];
		const float stepd = fabsf(t - tprev) * fabsf(D[2]);

		thfar = thick ? (stepd * 2.0f + 0.01f) : 0.0f;
		hit = ubf_probe(P0, D, winx, winy, winmat, depth, w, h, t, thfar);
		if (hit >= 0) {
			tnear = tprev;
			tfar = t;
			break;
		}
		tprev = t;
	}

	/* Бисекция — только когда пересечение было: уточнять нечего, если марш не
	 * нашёл ничего (иначе пробы в t = 0 на каждом промахе — впустую). */
	if (hit >= 0) {
		for (k = 0; k < bisect; k++) {
			const float tm = 0.5f * (tnear + tfar);
			const int jm = ubf_probe(P0, D, winx, winy, winmat, depth, w, h, tm, thfar);

			if (jm >= 0) {
				tfar = tm;
				hit = jm;
			}
			else {
				tnear = tm;
			}
		}
	}

	if (hit >= 0) hit_out[idx] = hit + 1;
	else if (defer) miss_out[idx] = 1;
}

/* ------------------------------------------------------------ хост-часть --- */

/* Буферы устройства: держим между кадрами, перевыделяем только при смене
 * размеров (копия G-буфера на 400x192 — 4.7 МБ, на 1920x1080 — 128 МБ; это и
 * есть та цена, которую надо замерить). */
typedef struct UBFGpuBuf {
	float *pos, *nrm, *view, *mir, *depth, *winmat, *ts;
	int *hit;
	unsigned char *miss;
	int n, ts_cap;
} UBFGpuBuf;

static UBFGpuBuf g_buf;
static int g_state = -2;      /* -2 не проверяли, -1 нет CUDA, 0 выключено env, 1 готово */
static char g_info[256] = "";

static void ubf_cuda_print_info(void)
{
	int dev = 0, driver = 0;
	cudaDeviceProp p;

	if (cudaGetDeviceProperties(&p, dev) != cudaSuccess) return;
	cudaDriverGetVersion(&driver);
	snprintf(g_info, sizeof(g_info), "%s, cc=%d.%d, sm=%d, mem=%d МБ",
	         p.name, p.major, p.minor, p.multiProcessorCount,
	         (int)(p.totalGlobalMem >> 20));
	printf("[UBF-CUDA] устройство: %s\n", g_info);
	printf("[UBF-CUDA] рантайм CUDA %d, драйвер %d\n", (int)CUDART_VERSION, driver);
	fflush(stdout);
}

int UBF_cuda_available(void)
{
	int count = 0;

	if (g_state == -2) {
		const char *e = getenv("UBF_BI_SSR_GPU");

		if (e == NULL || e[0] == '0') {
			g_state = 0;
			return 0;
		}
		if (cudaGetDeviceCount(&count) != cudaSuccess || count <= 0) {
			printf("[UBF-CUDA] устройств CUDA нет — марш остаётся на CPU\n");
			fflush(stdout);
			g_state = -1;
			return 0;
		}
		if (cudaSetDevice(0) != cudaSuccess) {
			g_state = -1;
			return 0;
		}
		ubf_cuda_print_info();
		g_state = 1;
	}
	return g_state == 1;
}

void UBF_cuda_info(char *buf, int len)
{
	if (buf == NULL || len <= 0) return;
	strncpy(buf, g_info, (size_t)len - 1);
	buf[len - 1] = '\0';
}

/* Выделение/перевыделение буферов под n пикселей и steps времён. */
static int ubf_cuda_ensure(int n, int steps)
{
	if (g_buf.n != n) {
		if (g_buf.pos) {
			cudaFree(g_buf.pos); cudaFree(g_buf.nrm); cudaFree(g_buf.view);
			cudaFree(g_buf.mir); cudaFree(g_buf.depth); cudaFree(g_buf.hit);
			cudaFree(g_buf.miss);
			memset(&g_buf, 0, sizeof(g_buf));
		}
		if (cudaMalloc((void **)&g_buf.pos, sizeof(float) * 3 * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.nrm, sizeof(float) * 3 * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.view, sizeof(float) * 3 * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.mir, sizeof(float) * 4 * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.depth, sizeof(float) * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.hit, sizeof(int) * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.miss, sizeof(unsigned char) * n) != cudaSuccess) return 1;
		if (cudaMalloc((void **)&g_buf.winmat, sizeof(float) * 16) != cudaSuccess) return 1;
		g_buf.n = n;
	}
	if (g_buf.ts_cap < steps) {
		if (g_buf.ts) cudaFree(g_buf.ts);
		if (cudaMalloc((void **)&g_buf.ts, sizeof(float) * steps) != cudaSuccess) return 1;
		g_buf.ts_cap = steps;
	}
	return 0;
}

int UBF_cuda_ssr_march(const UBFSSRGpu *job)
{
	cudaEvent_t ev0, ev1;
	float kernel_ms = 0.0f, copy_ms = 0.0f;
	int n, rc = 0;
	const int threads = 256;
	const int blocks = (job->w * job->h + threads - 1) / threads;

	if (!UBF_cuda_available()) return 1;

	n = job->w * job->h;
	if (ubf_cuda_ensure(n, job->steps) != 0) {
		printf("[UBF-CUDA] не удалось выделить память устройства\n");
		fflush(stdout);
		return 2;
	}

	cudaEventCreate(&ev0);
	cudaEventCreate(&ev1);

	/* Копии входов. */
	cudaEventRecord(ev0, 0);
	cudaMemcpy(g_buf.pos, job->pos, sizeof(float) * 3 * n, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.nrm, job->nrm, sizeof(float) * 3 * n, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.view, job->view, sizeof(float) * 3 * n, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.mir, job->mir, sizeof(float) * 4 * n, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.depth, job->depth, sizeof(float) * n, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.winmat, job->winmat, sizeof(float) * 16, cudaMemcpyHostToDevice);
	cudaMemcpy(g_buf.ts, job->ts, sizeof(float) * job->steps, cudaMemcpyHostToDevice);
	cudaMemset(g_buf.hit, 0, sizeof(int) * n);
	if (job->defer && job->miss) cudaMemset(g_buf.miss, 0, sizeof(unsigned char) * n);
	cudaEventRecord(ev1, 0);
	cudaEventSynchronize(ev1);
	cudaEventElapsedTime(&copy_ms, ev0, ev1);

	/* Само ядро. */
	cudaEventRecord(ev0, 0);
	ubf_ssr_march_kernel<<<blocks, threads>>>(
	    g_buf.pos, g_buf.nrm, g_buf.view, g_buf.mir, g_buf.depth,
	    job->winx, job->winy, g_buf.winmat, g_buf.ts,
	    job->w, job->h, job->steps, job->bisect, job->thick, job->defer,
	    g_buf.hit, g_buf.miss);
	cudaEventRecord(ev1, 0);
	cudaEventSynchronize(ev1);
	cudaEventElapsedTime(&kernel_ms, ev0, ev1);

	if (cudaGetLastError() != cudaSuccess) {
		printf("[UBF-CUDA] ошибка запуска ядра\n");
		fflush(stdout);
		rc = 3;
	}
	else {
		if (job->defer && job->miss) {
			cudaMemcpy(job->miss, g_buf.miss, sizeof(unsigned char) * n,
			           cudaMemcpyDeviceToHost);
		}
		cudaMemcpy(job->hit, g_buf.hit, sizeof(int) * n, cudaMemcpyDeviceToHost);
	}

	cudaEventDestroy(ev0);
	cudaEventDestroy(ev1);

	if (rc == 0) {
		printf("[UBF-CUDA] марш на GPU: %dx%d, шагов=%d, бисекция=%d: "
		       "ядро %.2f мс, копии %.2f мс\n",
		       job->w, job->h, job->steps, job->bisect, kernel_ms, copy_ms);
		fflush(stdout);
	}
	return rc;
}
