# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02")
  file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02")
endif()
file(MAKE_DIRECTORY
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/1"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/tmp"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/src/26_RC_02+26_RC_02-stamp"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/src"
  "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/src/26_RC_02+26_RC_02-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/src/26_RC_02+26_RC_02-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/spareE/Vinci_Robocon_2026/26_RC_Projects/26_RC_02_copy/MDK-ARM/tmp/26_RC_02+26_RC_02/src/26_RC_02+26_RC_02-stamp${cfgdir}") # cfgdir has leading slash
endif()
