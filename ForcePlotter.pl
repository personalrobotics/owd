#! /usr/bin/perl


# ***********************************************************************
# *                                                                     *
# * Copyright 2011 Carnegie Mellon University                           *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

$GP[0] = "GNUPLOT0";
open($GP[0] , "| gnuplot -geometry 1200x800");
# set autoflush
$GP[0]->autoflush(1);
# use lines for plots

$setup =<<EOM;
set style data lines
set y2tics
EOM
#set style line 1 lt 1 lc rgbcolor \"brown\"
#set style line 2 lt 1 lc rgbcolor \"red\"
#set style line 3 lt 1 lc rgbcolor \"orange\"
#set style line 4 lt 1 lc rgbcolor \"yellow\"
#set style line 5 lt 1 lc rgbcolor \"green\"
#set style line 6 lt 1 lc rgbcolor \"blue\"
#set style line 7 lt 1 lc rgbcolor \"violet\"
#
#set style line 8 lt 0 lc rgbcolor \"brown\"
#set style line 9 lt 0 lc rgbcolor \"red\"
#set style line 10 lt 0 lc rgbcolor \"orange\"
#set style line 11 lt 0 lc rgbcolor \"yellow\"
#set style line 12 lt 0 lc rgbcolor \"green\"
#set style line 13 lt 0 lc rgbcolor \"blue\"
#set style line 14 lt 0 lc rgbcolor \"violet\"
#
#set style line 22 lt 1 lc rgbcolor \"black\"

print {$GP[0]} $setup;

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
	print {$GP[0]} "set title \"Force readings in WS coords ($name)\"\n";
	print {$GP[0]} "  plot $f using 49 title \"X force\" with lines ls 1\n";
	print {$GP[0]} "replot $f using 50 title \"Y force\" with lines ls 2\n";
	print {$GP[0]} "replot $f using 51 title \"Z force\" with lines ls 3\n";
	print {$GP[0]} "replot $f using 52 title \"raw X force\" with lines ls 4\n";
	print {$GP[0]} "replot $f using 53 title \"raw Y force\" with lines ls 5\n";
	print {$GP[0]} "replot $f using 54 title \"raw Z force\" with lines ls 6\n";
	print {$GP[0]} "replot $f using 1 title \"time factor (y2)\" with lines ls 22 axes x1y2\n";


    } else {
	print "$f not found";
	print {$GP[0]} "clear\n";
    }
}


close {$GP[0]};
