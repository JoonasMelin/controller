#!/usr/bin/env python3
'''
Basic animation test case for Host-side KLL
'''

# Copyright (C) 2017 by Jacob Alexander
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.

### Imports ###

import interface as i

from common import (ERROR, WARNING, check, result)



### Test ###

# Reference to callback datastructure
data = i.control.data

# Drop to cli, type exit in the displayed terminal to continue
#i.control.cli()
i.control.cmd('addAnimation')(name='underlighting_only_rotation', pfunc=1, loops=1) # TODO


# Show output
i.control.cmd('rectDisp')()
#for index in range( 15 ):
#	i.control.cmd('addScanCode')( 0x3C )
#	i.control.loop(1)
#	i.control.cmd('removeScanCode')( 0x3C )
#	i.control.loop(1)

# Loop 13 times, displaying each time
for index in range( 12 ):
#for index in range( 1 ):
	# Read animation stack info
	print( "Loop {0} - Expecting Stack Size: 1 Got: {1}".format( index, i.control.cmd('animationStackInfo')().size ) )
	check( i.control.cmd('animationStackInfo')().size == 1 )

	# Loop once
	i.control.cmd('setFrameState')(2)
	i.control.loop(1)

	# Show output
	i.control.cmd('rectDisp')()

#i.control.cmd('removeScanCode')( 0x3C )
i.control.loop(1)

if False:

	print("-Rainbow Animation Test-");

	# Add Animation, index 5, to Stack (z2_rainbow_fill_interp)
	i.control.cmd('addAnimation')(index=2, pfunc=1) # TODO

	# Loop 13 times, displaying each time
	for index in range( 2 ):
		# Read animation stack info
		print( "Loop {0} - Expecting Stack Size: 1 Got: {1}".format( index, i.control.cmd('animationStackInfo')().size ) )
		check( i.control.cmd('animationStackInfo')().size == 1 )

		# Loop once
		i.control.cmd('setFrameState')(2)
		i.control.loop(1)

		# Show output
		i.control.cmd('rectDisp')()


	i.control.cmd('addAnimation')(index=10, loops=30, pfunc=1) # TODO
	for index in range( 30 ):
		# Read animation stack info
		print( "Loop {0} - Expecting Stack Size: 1 Got: {1}".format( index, i.control.cmd('animationStackInfo')().size ) )
		check( i.control.cmd('animationStackInfo')().size == 1 )

		# Loop once
		i.control.cmd('setFrameState')(2)
		i.control.loop(1)

		# Show output
		i.control.cmd('rectDisp')()

	# Loop one more time to clear stack
	#i.control.cmd('setFrameState')(2)
	#i.control.loop(1)

	# Read animation stack info
	#print( "Final - Expecting Stack Size: 0 Got:", i.control.cmd('animationStackInfo')().size )
	#check( i.control.cmd('animationStackInfo')().size == 0 )


##### Tests Complete #####

result()

