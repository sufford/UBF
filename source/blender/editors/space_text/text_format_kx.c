/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file blender/editors/space_text/text_format_kx.c
 *  \ingroup sptext
 */

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"

#include "DNA_space_types.h"
#include "DNA_text_types.h"

#include "text_format.h"

#include <ctype.h>    // For isdigit, isalpha
#include <string.h>   // For memcpy, strcmp, strchr

/*************************/
/* KXScript syntax highlighting */
/*************************/

static bool txtfmt_kx_is_comment(const char *str)
{
	return str[0] == '#' || (str[0] == '/' && str[1] == '/');
}

static bool txtfmt_kx_is_string(const char *str)
{
	return str[0] == '"';
}

static void txtfmt_kx_format_line(SpaceText *st, TextLine *line, const bool do_next)
{
	FlattenString fs;
	const char *str;
	char *fmt;
	char cont_orig, ch, prev;
	int len;

	if (*line->line == '\0') {
		return;
	}

	flatten_string(st, &fs, line->line);
	str = fs.buf;
	cont_orig = ch = *str;
	prev = 0;
	len = flatten_string_strlen(&fs, str);
	fmt = MEM_mallocN(len + 3, "SyntaxFormat");
	text_check_format_len(line, len);
	line->format = fmt;

	while (*str) {
		/* Handle comments */
		if (txtfmt_kx_is_comment(str)) {
			text_format_fill_ascii(&str, &fmt, FMT_TYPE_COMMENT, len - (int)(fmt - line->format));
			break;
		}
		/* Handle strings */
		else if (txtfmt_kx_is_string(str)) {
			text_format_fill_ascii(&str, &fmt, FMT_TYPE_STRING, len - (int)(fmt - line->format));
			break;
		}
		/* Handle numbers */
		else if (isdigit(*str)) {
			const char *start = str;
			while (isdigit(*str) || *str == '.') {
				str++;
			}
			int num_len = str - start;
			text_format_fill_ascii(&start, &fmt, FMT_TYPE_NUMERAL, num_len);
			str--;
			fmt--;
		}
		/* Handle keywords and identifiers */
		else if (isalpha(*str) || *str == '_') {
			const char *start = str;
			while (isalpha(*str) || isdigit(*str) || *str == '_') {
				str++;
			}
			
			int word_len = str - start;
			char *word = MEM_mallocN(word_len + 1, "keyword_check");
			memcpy(word, start, word_len);
			word[word_len] = '\0';
			
			/* Check if it's a keyword */
			const char *keywords[] = {
				"var", "if", "else", "while", "func",
				"return", "true", "false", "print",
				NULL
			};
			
			bool is_keyword = false;
			for (int i = 0; keywords[i]; i++) {
				if (STREQ(word, keywords[i])) {
					is_keyword = true;
					break;
				}
			}
			
			if (is_keyword) {
				text_format_fill_ascii(&start, &fmt, FMT_TYPE_KEYWORD, word_len);
			} else {
				text_format_fill_ascii(&start, &fmt, FMT_TYPE_DEFAULT, word_len);
			}
			
			MEM_freeN(word);
			str--;
			fmt--;
		}
		/* Handle brackets and symbols */
		else if (strchr("()[]{}.,;:+-*/=<>!$", *str)) {
			text_format_fill_ascii(&str, &fmt, FMT_TYPE_SYMBOL, 1);
		}
		/* Handle whitespace */
		else if (*str == ' ' || *str == '\t') {
			text_format_fill_ascii(&str, &fmt, FMT_TYPE_WHITESPACE, 1);
		}
		/* Handle default */
		else {
			text_format_fill_ascii(&str, &fmt, FMT_TYPE_DEFAULT, 1);
		}
	}

	flatten_string_free(&fs);
}

static const char *ext_kx[] = {".kx", NULL};

static TextFormatType tft_kx = {
	NULL,  /* next */
	NULL,  /* prev */
	NULL,  /* format_identifier */
	txtfmt_kx_format_line,
	ext_kx,
};

void ED_text_format_register_kx(void)
{
	ED_text_format_register(&tft_kx);
}