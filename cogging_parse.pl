#!/usr/bin/perl

$lasttorque=0;
$possum=0;
$poscount=0;
while (<>) {
    if (/position: (.*)/) {
	$position=$1;
    }
    if (/torque: (.*)/) {
	$torque = $1;
	if ($torque == $lasttorque) {
	    $possum+=$position;
	    $poscount++;
	} else {
	    $avgpos=$possum/$poscount;
	    print "$avgpos $lasttorque $poscount\n";
	    $possum=$position;
	    $poscount=1;
	    $lasttorque=$torque;
	}
    }
}

$avgpos=$possum/$poscount;
print "$avgpos $lasttorque $poscount\n";
