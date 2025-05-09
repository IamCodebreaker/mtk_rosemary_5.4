/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton interface for Bison's Yacc-like parsers in C

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     DT_V1 = 258,
     DT_PLUGIN = 259,
     DT_MEMRESERVE = 260,
     DT_LSHIFT = 261,
     DT_RSHIFT = 262,
     DT_LE = 263,
     DT_GE = 264,
     DT_EQ = 265,
     DT_NE = 266,
     DT_AND = 267,
     DT_OR = 268,
     DT_BITS = 269,
     DT_DEL_PROP = 270,
     DT_DEL_NODE = 271,
     DT_OMIT_NO_REF = 272,
     DT_PROPNODENAME = 273,
     DT_LITERAL = 274,
     DT_CHAR_LITERAL = 275,
     DT_BYTE = 276,
     DT_STRING = 277,
     DT_LABEL = 278,
     DT_LABEL_REF = 279,
     DT_PATH_REF = 280,
     DT_INCBIN = 281
   };
#endif
/* Tokens.  */
#define DT_V1 258
#define DT_PLUGIN 259
#define DT_MEMRESERVE 260
#define DT_LSHIFT 261
#define DT_RSHIFT 262
#define DT_LE 263
#define DT_GE 264
#define DT_EQ 265
#define DT_NE 266
#define DT_AND 267
#define DT_OR 268
#define DT_BITS 269
#define DT_DEL_PROP 270
#define DT_DEL_NODE 271
#define DT_OMIT_NO_REF 272
#define DT_PROPNODENAME 273
#define DT_LITERAL 274
#define DT_CHAR_LITERAL 275
#define DT_BYTE 276
#define DT_STRING 277
#define DT_LABEL 278
#define DT_LABEL_REF 279
#define DT_PATH_REF 280
#define DT_INCBIN 281




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE

{
	char *propnodename;
	char *labelref;
	uint8_t byte;
	struct data data;

	struct {
		struct data	data;
		int		bits;
	} array;

	struct property *prop;
	struct property *proplist;
	struct node *node;
	struct node *nodelist;
	struct reserve_info *re;
	uint64_t integer;
	unsigned int flags;
}
/* Line 1529 of yacc.c.  */

	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE yylval;

#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
} YYLTYPE;
# define yyltype YYLTYPE /* obsolescent; will be withdrawn */
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif

extern YYLTYPE yylloc;
