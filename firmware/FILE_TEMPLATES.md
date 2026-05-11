# Firmware File Templates

Use these templates when creating new `.h` and `.c` files in `firmware/`.

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
