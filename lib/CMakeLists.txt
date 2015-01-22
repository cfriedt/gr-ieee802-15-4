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

#####################################################################
# Setup library
#####################################################################
include(GrPlatform) #define LIB_SUFFIX

list(APPEND gr_ieee802154_sources
    access_code_prefixer.cc
    bc_connection.cc
    mac.cc
    packet_sink.cc
    rime_connection.cc
    rime_stack.cc
    ruc_connection.cc
    stubborn_sender.cc
    uc_connection.cc
)

list(APPEND gr_ieee802154_libs
    ${Boost_LIBRARIES}
    ${GNURADIO_ALL_LIBRARIES}
)

add_library(gnuradio-ieee802154 SHARED ${gr_ieee802154_sources})
target_link_libraries(gnuradio-ieee802154 ${gr_ieee802154_libs})
set_target_properties(gnuradio-ieee802154 PROPERTIES DEFINE_SYMBOL "gnuradio_ieee802154_EXPORTS")

#####################################################################
# Install built library files
#####################################################################
install(TARGETS gnuradio-ieee802154
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)
