# Firmware File Templates

Use these templates when creating new `.h` and `.c` files in `firmware/`.

## Documentation And Initialization Rules

- Initialize every variable at declaration. Local variables inside functions are exempt only from the Doxygen requirement, not from initialization.
- Add Doxygen comments for functions, types, global variables, file-scope variables, macros, and macro definitions.
- Put each function Doxygen block next to the prototype/declaration. Public function comments belong before header prototypes; private `static` function comments belong before source-file prototypes.
- Do not duplicate the same Doxygen block above the implementation if the prototype already has it.
- Any Doxygen comment that contains `\brief` must use the multi-line block form, even when the brief is the only tag. Single-line `/** \brief ... */` comments are not allowed. Trailing `/**< ... */` comments on the same line as a field, macro, or variable declaration remain allowed and are not required to be expanded.
- Function and function-like macro comments must document every parameter and the return value (when the function returns a value). Do not omit `\param` for any parameter, and do not omit `\return` for non-`void` returns.
- Every `\param` entry must declare the parameter direction with `\param[in]`, `\param[out]`, or `\param[in,out]`. Bare `\param name` without a direction is not allowed.

## Doxygen Snippets

Macro:

```c
#define <MACRO_NAME> (<VALUE>) /**< Brief macro description. */
```

Type:

```c
/**
 * \brief Brief type description.
 */
typedef struct {
	uint32_t field; /**< Brief field description. */
} <type_name>_t;
```

Global or file-scope data:

```c
extern uint32_t <global_name>; /**< Brief global variable description. */
static uint32_t <private_value> = 0; /**< Brief file-scope variable description. */
```

Function prototype:

```c
/**
 * \brief Brief function description.
 *
 * \param[in] value Brief input parameter description.
 * \param[out] result Brief output parameter description.
 * \return Brief return value description.
 */
bool <function_name>(uint32_t value, uint32_t *result);
```

Function-like macro:

```c
/**
 * \brief Brief macro description.
 *
 * \param[in] value Brief input parameter description.
 * \return Brief return value description.
 */
#define <MACRO_NAME>(value) (/* expression */)
```

## Choosing A Header

- Use "Original MioLink File Header" for files created specifically for MioLink.
- Use "Black Magic-Derived File Header" for files copied or adapted from the Black Magic Debug project.
- For Black Magic-derived files, keep all previous copyright lines from the source file where `<KEEP ALL previous copyrights>` appears. Do not replace upstream copyright history with only the MioLink modification line.
- Keep the license block at the very top of the file.

## Original MioLink File Header

```c
/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2026 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
```

## Black Magic-Derived File Header

```c
/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * <KEEP ALL previous copyrights>
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
```

## Header File Template

Replace `<MODULE_NAME>` with the uppercase module token used for the include guard, for example `MIOLINK_USB_CDC`.

Omit any section whose content would be empty.

```c
#ifndef <MODULE_NAME>_H
#define <MODULE_NAME>_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

#endif /* <MODULE_NAME>_H */
```

## Source File Template

Omit any section whose content would be empty.

```c
/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * External Data
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/
```
