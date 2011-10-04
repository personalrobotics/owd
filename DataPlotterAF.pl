#! /usr/bin/perl


# ***********************************************************************
# *                                                                     *
# * Copyright 2011 Carnegie Mellon University                           *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

$GP[0] = "GNUPLOT0";
$GP[1] = "GNUPLOT1";
$GP[2] = "GNUPLOT2";
open($GP[0] , "| gnuplot -geometry 1200x800");
open($GP[1] , "| gnuplot -geometry 1200x800");
open($GP[2] , "| gnuplot -geometry 1200x800");
# set autoflush
$GP[0]->autoflush(1);
$GP[1]->autoflush(1);
$GP[2]->autoflush(1);
# use lines for plots

$setup =<<EOM;
set style data lines
set y2tics
set style line 1 lt 1 lc rgbcolor \"brown\"
set style line 2 lt 1 lc rgbcolor \"red\"
set style line 3 lt 1 lc rgbcolor \"orange\"
set style line 4 lt 1 lc rgbcolor \"yellow\"
set style line 5 lt 1 lc rgbcolor \"green\"
set style line 6 lt 1 lc rgbcolor \"blue\"
set style line 7 lt 1 lc rgbcolor \"violet\"

set style line 8 lt 0 lc rgbcolor \"brown\"
set style line 9 lt 0 lc rgbcolor \"red\"
set style line 10 lt 0 lc rgbcolor \"orange\"
set style line 11 lt 0 lc rgbcolor \"yellow\"
set style line 12 lt 0 lc rgbcolor \"green\"
set style line 13 lt 0 lc rgbcolor \"blue\"
set style line 14 lt 0 lc rgbcolor \"violet\"

set style line 22 lt 1 lc rgbcolor \"black\"
EOM

print {$GP[0]} $setup;
print {$GP[1]} $setup;
print {$GP[2]} $setup;

$linenum = -1;
while (<>) {
    if (/b|B|p/) { # back / previous
	$linenum = $linenum-1;
	if ($linenum < 0) {
	    $linenum = 0;
	}
    } elsif (/(\d+)/) {
	$linenum = $1;
    } elsif (/LP/) {
	print {$GP[1]} "set style data linespoints\n";
    } elsif (/P/) {
	print {$GP[1]} "set style data points\n";
    } elsif (/L/) {
	print {$GP[1]} "set style data lines\n";
    } else {
	$linenum = $linenum+1;
    }
    $f = sprintf("gfeplugin-%04d.csv",$linenum);
    print "Plotting $f\n";
    $name=$f;

    if (-e $f) {
	$f = "\"$f\"";
#	print {$GP[0]} "set title \"Force reading and error in WS coords ($name)\"\n";
#	print {$GP[0]} "  plot $f using 49 title \"X force\" with lines ls 1\n";
#	print {$GP[0]} "replot $f using 50 title \"Y force\" with lines ls 2\n";
#	print {$GP[0]} "replot $f using 51 title \"Z force\" with lines ls 3\n";
#	print {$GP[0]} "replot $f using 5 title \"X error\" with lines ls 8\n";
#	print {$GP[0]} "replot $f using 6 title \"X error\" with lines ls 9\n";
#	print {$GP[0]} "replot $f using 7 title \"X error\" with lines ls 10\n";
#	print {$GP[0]} "replot $f using 1 title \"time factor\" with lines ls 22 axes x1y2\n";


	print {$GP[1]} "set title \"Joint torques to correct force ($name)\"\n";
	print {$GP[1]} "  plot $f using 8 title \"J1 torque\" with lines ls 1\n";
	print {$GP[1]} "replot $f using 9 title \"J2 torque\" with lines ls 2\n";
	print {$GP[1]} "replot $f using 10 title \"J3 torque\" with lines ls 3\n";
	print {$GP[1]} "replot $f using 11 title \"J4 torque\" with lines ls 4\n";
	print {$GP[1]} "replot $f using 12 title \"J5 torque\" with lines ls 5\n";
	print {$GP[1]} "replot $f using 13 title \"J6 torque\" with lines ls 6\n";
	print {$GP[1]} "replot $f using 14 title \"J7 torque\" with lines ls 7\n";
	print {$GP[1]} "replot $f using 1 title \"time factor\" with lines ls 22 axes x1y2\n";



	print {$GP[0]} "set title \"Joint corrections ($name)\"\n";
	print {$GP[0]} "  plot $f using 42 title \"J1 delta\" with lines ls 1\n";
	print {$GP[0]} "replot $f using 43 title \"J2 delta\" with lines ls 2\n";
	print {$GP[0]} "replot $f using 44 title \"J3 delta\" with lines ls 3\n";
	print {$GP[0]} "replot $f using 45 title \"J4 delta\" with lines ls 4\n";
	print {$GP[0]} "replot $f using 46 title \"J5 delta\" with lines ls 5\n";
	print {$GP[0]} "replot $f using 47 title \"J6 delta\" with lines ls 6\n";
	print {$GP[0]} "replot $f using 48 title \"J7 delta\" with lines ls 7\n";
	print {$GP[0]} "replot $f using 1 title \"time factor\" with lines ls 22 axes x1y2\n";


#	print {$GP[1]} "set title \"Positional errors ($name)\"\n";
#	print {$GP[1]} "  plot $f using 15 title \"X error\"  with lines ls 1\n";
#	print {$GP[1]} "replot $f using 16 title \"Y error\" with lines ls 2\n";
#	print {$GP[1]} "replot $f using 17 title \"Z error\" with lines ls 3\n";
#	print {$GP[1]} "replot $f using 18 title \"r error\" with lines ls 4 axes x1y2\n";
#	print {$GP[1]} "replot $f using 19 title \"p error\" with lines ls 5 axes x1y2\n ";
#	print {$GP[1]} "replot $f using 20 title \"y error\" with lines ls 6 axes x1y2\n";

	print {$GP[2]} "set title \"PID torques from previous cycle ($name)\"\n";
	print {$GP[2]} "  plot $f using 35 title \"J1 PID\" with lines ls 1\n";
	print {$GP[2]} "replot $f using 36 title \"J2 PID\" with lines ls 2\n";
	print {$GP[2]} "replot $f using 37 title \"J3 PID\" with lines ls 3\n";
	print {$GP[2]} "replot $f using 38 title \"J4 PID\" with lines ls 4\n";
	print {$GP[2]} "replot $f using 39 title \"J5 PID\" with lines ls 5\n";
	print {$GP[2]} "replot $f using 40 title \"J6 PID\" with lines ls 6\n";
	print {$GP[2]} "replot $f using 41 title \"J7 PID\" with lines ls 7\n";

#	print {$GP[0]} "set title $f\n";
#	print {$GP[0]} "  plot $f using 28 title \"J1 input\" with lines ls 1\n";
#	print {$GP[0]} "replot $f using 29 title \"J2 input\" with lines ls 2\n";
#	print {$GP[0]} "replot $f using 30 title \"J3 input\" with lines ls 3\n";
#	print {$GP[0]} "replot $f using 31 title \"J4 input\" with lines ls 4\n";
#	print {$GP[0]} "replot $f using 32 title \"J5 input\" with lines ls 5\n";
#	print {$GP[0]} "replot $f using 33 title \"J6 input\" with lines ls 6\n";
#	print {$GP[0]} "replot $f using 34 title \"J7 input\" with lines ls 7\n";

#	print {$GP[0]} "set title $f\n";
#	print {$GP[0]} "  plot $f using 21 title \"J1 correction\" with lines ls 1\n";
#	print {$GP[0]} "replot $f using 22 title \"J2 correction\" with lines ls 2\n";
#	print {$GP[0]} "replot $f using 23 title \"J3 correction\" with lines ls 3\n";
#	print {$GP[0]} "replot $f using 24 title \"J4 correction\" with lines ls 4\n";
#	print {$GP[0]} "replot $f using 25 title \"J5 correction\" with lines ls 5\n";
#	print {$GP[0]} "replot $f using 26 title \"J6 correction\" with lines ls 6\n";
#	print {$GP[0]} "replot $f using 27 title \"J7 correction\" with lines ls 7\n";

    } else {
	print "$f not found";
	print {$GP[0]} "clear\n";
	print {$GP[1]} "clear\n";
	print {$GP[2]} "clear\n";
    }
}


close {$GP[0]};
