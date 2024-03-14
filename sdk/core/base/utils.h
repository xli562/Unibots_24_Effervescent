
#pragma once

#ifdef WIN32
#ifdef oradar_IMPORTS
#define ORDLIDAR_API __declspec(dllimport)
#else
#ifdef oradarStatic_IMPORTS
#define ORDLIDAR_API
#else

#define ORDLIDAR_API __declspec(dllexport)
#endif // ORDLIDAR_STATIC_EXPORTS
#endif

#else
#define ORDLIDAR_API
#endif // ifdef WIN32
