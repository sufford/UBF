/*
 * ubf_cuda.h — интерфейс CUDA-бэкенда UBF (этап A1: марш экранных отражений).
 *
 * Задача этапа: доказать, что GPU-счёт в этой сборке (а) собирается, (б) даёт
 * БИТ-В-БИТ тот же результат, что CPU-эталон, и только потом переносить на GPU
 * что-то ещё. Поэтому наружу торчит узкая функция: «посчитай попадания марша и
 * маску промахов по G-буферу кадра». Всё остальное (смешение, ремонтный проход,
 * слияние частей) остаётся на CPU — тогда сравнение картинок CPU-пути и
 * GPU-пути проверяет ровно ядро, а не всю систему.
 *
 * Точность: ядро компилируется с -fmad=false (никакой склейки mul+add), таблица
 * t считается на CPU и передаётся готовой — поэтому все float-операции в ядре
 * повторяют CPU-эталон по порядку и округлению.
 *
 * Сборка: только при WITH_UBF_CUDA (по умолчанию ВЫКЛ). Включение в рантайме —
 * переменная UBF_BI_SSR_GPU=1 (только BI).
 */

#ifndef __UBF_CUDA_H__
#define __UBF_CUDA_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WITH_UBF_CUDA

/* Задание на марш. Массивы — те же, что читает CPU-эталон:
 *   pos/nrm/view — по 3 float на пиксель, mir — 4 (0: i, 1..3: mirror_color),
 *   depth — 1 float на пиксель, ts — таблица времён шага (steps штук).
 * Результат: hit[idx] = индекс пикселя-попадания + 1 (0 = промах);
 * miss[idx] = 1, если промах и режим defer (иначе не пишется). */
typedef struct UBFSSRGpu {
	const float *pos;
	const float *nrm;
	const float *view;
	const float *mir;
	const float *depth;
	const float *winmat;      /* 16 float, как re->winmat */
	const float *ts;          /* steps float: t для k = 1..steps */
	int w, h;
	float winx, winy;
	int steps;
	int bisect;               /* 0 = без бисекции */
	int thick;                /* 1 = тест попадания «с толщиной» */
	int defer;                /* 1 = писать маску промахов */
	int *hit;                 /* out, w*h int */
	unsigned char *miss;      /* out, w*h uchar (только при defer) */
} UBFSSRGpu;

/* 1 — CUDA собрана и устройство доступно. Печатает [UBF-CUDA] один раз. */
int UBF_cuda_available(void);

/* Текстовое описание устройства (для лога). */
void UBF_cuda_info(char *buf, int len);

/* Марш на GPU. 0 — успех, иначе код ошибки (и тогда CPU обязан посчитать сам,
 * чтобы кадр не пострадал). */
int UBF_cuda_ssr_march(const UBFSSRGpu *job);

#endif /* WITH_UBF_CUDA */

#ifdef __cplusplus
}
#endif

#endif /* __UBF_CUDA_H__ */
