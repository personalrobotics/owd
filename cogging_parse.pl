#!/usr/bin/perl

# set OFFSET equal to the motor offset reported by OWD at startup.  It
# should be in units of encoder ticks.  The offset is the angle that is
# ADDED to the MECH value to get the puck AP value.  The CoggingComp
# trajectory records AP values, so to recover the true MECH value we will
# subtract the offset.  If you have updated the CoggingComp trajectory
# to report true MECH values then set OFFSET equal to zero.
$offset = -985;

# set AVERAGE=0 to just dump out all the raw values in order to look for
# drift.  set AVERAGE=1 to average together all the samples for a single
# torque value.
$AVERAGE=1;



$TWO_PI = 2.0 * 3.141592654;
$offset_radians = $offset / 4096 * $TWO_PI;
$lasttorque=0;
$possum=0;
$poscount=0;
while (<>) {
    if (/position: (.*)/) {
	$position=$1 - $offset_radians;
    }
    if (/torque: (.*)/) {
	$torque = $1;
	if (! $AVERAGE) {
	    print "$position $torque\n";
	} else {
	    if ($torque == $lasttorque) {
		$possum+=$position;
		$poscount++;
	    } else {
		$avgpos=$possum/$poscount;
		# map to the 0-2PI range
		while ($avgpos < 0) {
		    $avgpos += $TWO_PI;
		}
		while ($avgpos > $TWO_PI) {
		    $avgpos -= $TWO_PI;
		}
		print "$avgpos $lasttorque $poscount\n";
		$possum=$position;
		$poscount=1;
		$lasttorque=$torque;
	    }
	}
    }
}

if ($AVERAGE) {
    # print the final group
    $avgpos=$possum/$poscount;

    # map to the 0-2PI range
    while ($avgpos < 0) {
	$avgpos += $TWO_PI;
    }
    while ($avgpos > $TWO_PI) {
	$avgpos -= $TWO_PI;
    }
    print "$avgpos $lasttorque $poscount\n";
}
