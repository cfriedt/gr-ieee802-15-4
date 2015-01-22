#
# Copyright 2013 Free Software Foundation, Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# Include swig generation macros
########################################################################
include(GrPython)
find_package(SWIG)
find_package(PythonLibs)
if(NOT SWIG_FOUND OR NOT PYTHONLIBS_FOUND)
    return()
endif()
include(GrSwig)

########################################################################
# Setup swig generation
########################################################################
set(GR_SWIG_INCLUDE_DIRS
    ${GR_IEEE802154_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

foreach(incdir ${GNURADIO_RUNTIME_INCLUDE_DIRS})
    list(APPEND GR_SWIG_INCLUDE_DIRS ${incdir}/gnuradio/swig)
endforeach(incdir)

set(GR_SWIG_LIBRARIES gnuradio-ieee802154)
set(GR_SWIG_DOC_FILE ${CMAKE_CURRENT_BINARY_DIR}/ieee802154_swig_doc.i)
set(GR_SWIG_DOC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include/ieee802154)

GR_SWIG_MAKE(ieee802154_swig ieee802154_swig.i)

########################################################################
# Install the build swig module
########################################################################
GR_SWIG_INSTALL(
    TARGETS ieee802154_swig
    DESTINATION ${GR_PYTHON_DIR}/ieee802154
    COMPONENT "ieee802154_python"
)

########################################################################
# Install swig .i files for development
########################################################################
list(APPEND swig_sources
    ieee802154_swig.i
    ${CMAKE_CURRENT_BINARY_DIR}/ieee802154_swig_doc.i
)

install(
    FILES ${swig_sources}
    DESTINATION ${GR_INCLUDE_DIR}/ieee802154/swig
    COMPONENT "ieee802154_swig"
)
