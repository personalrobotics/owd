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
open($GP[0] , "| gnuplot -geometry 600x280");
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[0]->autoflush(1);
$GP[1]->autoflush(1);
# use lines for plots

$setup =<<EOM;
set style data lines
set y2tics
set style line 1 lt 0 lc rgbcolor \"brown\"
set style line 2 lt 0 lc rgbcolor \"red\"
set style line 3 lt 0 lc rgbcolor \"orange\"
set style line 4 lt 0 lc rgbcolor \"yellow\"
set style line 5 lt 0 lc rgbcolor \"green\"
set style line 6 lt 0 lc rgbcolor \"blue\"
set style line 7 lt 0 lc rgbcolor \"violet\"

set style line 8 lt 1 lc rgbcolor \"brown\"
set style line 9 lt 1 lc rgbcolor \"red\"
set style line 10 lt 1 lc rgbcolor \"orange\"
set style line 11 lt 1 lc rgbcolor \"yellow\"
set style line 12 lt 1 lc rgbcolor \"green\"
set style line 13 lt 1 lc rgbcolor \"blue\"
set style line 14 lt 1 lc rgbcolor \"violet\"

set style line 15 lt 2 lc rgbcolor \"brown\"
set style line 16 lt 2 lc rgbcolor \"red\"
set style line 17 lt 2 lc rgbcolor \"orange\"
set style line 18 lt 2 lc rgbcolor \"yellow\"
set style line 19 lt 2 lc rgbcolor \"green\"
set style line 20 lt 2 lc rgbcolor \"blue\"
set style line 21 lt 2 lc rgbcolor \"violet\"

set style line 22 lt 1 lc rgbcolor \"black\"
EOM

print {$GP[0]} $setup;
print {$GP[1]} $setup;

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
    $f = sprintf("doortraj-%02d.csv",$linenum);
    print "Plotting $f\n";
    
    if (-e $f) {
	$f = "\"$f\"";
	print {$GP[1]} "set title $f\n";
#print {$GP[1]} "replot $f using 3 title \"traj time\"\n"; #traj_time
	print {$GP[1]} "  plot $f using 1:9  title \"J1 traj\" with lines ls 15\n";
	print {$GP[1]} "replot $f using 1:74 title \"J1 targ\"  with lines ls 1\n";
	print {$GP[1]} "replot $f using 1:2  title \"J1 act\"  with lines ls 8\n";
	print {$GP[1]} "replot $f using 1:10 title \"J2 traj\" with lines ls 16\n";
	print {$GP[1]} "replot $f using 1:75 title \"J2 targ\" with lines ls 2\n";
	print {$GP[1]} "replot $f using 1:3  title \"J2 act\"  with lines ls 9\n";
	print {$GP[1]} "replot $f using 1:11 title \"J3 traj\" with lines ls 17\n";
	print {$GP[1]} "replot $f using 1:76 title \"J3 targ\" with lines ls 3\n";
	print {$GP[1]} "replot $f using 1:4  title \"J3 act\"  with lines ls 10\n";
	print {$GP[1]} "replot $f using 1:12 title \"J4 traj\" with lines ls 18\n";
	print {$GP[1]} "replot $f using 1:77 title \"J4 targ\" with lines ls 4\n";
	print {$GP[1]} "replot $f using 1:5  title \"J4 act\"  with lines ls 11\n";
	print {$GP[1]} "replot $f using 1:13 title \"J5 traj\" with lines ls 19\n";
	print {$GP[1]} "replot $f using 1:78 title \"J5 targ\" with lines ls 5\n ";
	print {$GP[1]} "replot $f using 1:6  title \"J5 act\"  with lines ls 12\n";
	print {$GP[1]} "replot $f using 1:14 title \"J6 traj\" with lines ls 20\n";
	print {$GP[1]} "replot $f using 1:79 title \"J6 targ\" with lines ls 6\n";
	print {$GP[1]} "replot $f using 1:7  title \"J6 act\"  with lines ls 13\n";
	print {$GP[1]} "replot $f using 1:15 title \"J7 traj\" with lines ls 21\n";
	print {$GP[1]} "replot $f using 1:80 title \"J7 targ\" with lines ls 7\n";
	print {$GP[1]} "replot $f using 1:8  title \"J7 act\"  with lines ls 14\n";

	print {$GP[0]} "set title $f\n";
	print {$GP[0]} "  plot $f using 1:19 title \"X traj\" with lines ls 1\n";
	print {$GP[0]} "replot $f using 1:54 title \"X long\" with lines ls 8\n";
	print {$GP[0]} "replot $f using 1:51 title \"X lat\" with lines  ls 11\n";
	print {$GP[0]} "replot $f using 1:23 title \"Y traj\" with lines ls 2\n";
	print {$GP[0]} "replot $f using 1:55 title \"Y long\" with lines ls 9\n";
	print {$GP[0]} "replot $f using 1:52 title \"Y lat\" with lines  ls 12\n";
	print {$GP[0]} "replot $f using 1:27 title \"Z traj\" with lines ls 3\n";
	print {$GP[0]} "replot $f using 1:56 title \"Z long\" with lines ls 10\n";
	print {$GP[0]} "replot $f using 1:53 title \"Z lat\" with lines  ls 13\n";
	print {$GP[0]} "replot $f using 1:57 axes x1y2 title \"rot X\" with lines ls 4\n";
	print {$GP[0]} "replot $f using 1:58 axes x1y2 title \"rot Y\" with lines ls 5\n";
	print {$GP[0]} "replot $f using 1:59 axes x1y2 title \"rot Z\" with lines ls 6\n";

    } else {
	print "$f not found";
	print {$GP[0]} "clear\n";
	print {$GP[1]} "clear\n";
    }
}


close {$GP[0]};
