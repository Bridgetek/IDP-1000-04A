/*
This file is automatically generated
person
Header
*/

#ifndef person__H
#define person__H

#ifndef ESD_FUNCTION
#define ESD_FUNCTION(name, ...)
#endif

#include "Ft_DataTypes.h"
#include "Ft_Esd_BitmapInfo.h"

Ft_Esd_BitmapCell person(ft_uint16_t cell);

extern Ft_Esd_BitmapInfo person__Info;

ESD_FUNCTION(person_0, Type = Ft_Esd_BitmapCell, DisplayName = "person", Include = "person.h", Category = _GroupUserResources, Icon = ":/icons/image.png", Macro)
#define person_0() ((Ft_Esd_BitmapCell){ .Info = &person__Info, .Cell = 0 })

#endif /* person__H */

/* end of file */
