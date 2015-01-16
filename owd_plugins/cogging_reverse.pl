#!/usr/bin/perl

# This script reverses the order of the cogs in the collected cogging
# data file. Run this on the collected data if the angles are not
# monotonically increasing or decreasing (e.g., cog climbs are moving
# in the positive direction but the cogs themselves were collected in
# the negative direction).
#
# Make an empty directory and cd to it before running this script.
# It will create a set of "cog###.txt" files that are numbered in
# reverse order.  Just "cat" all the files together and redirect the
# output to your new file, e.g.:
#     cat cog*.txt > ../cogging_motor_data_ccw_reversed.txt



$filenum=999;
$filestr="999";


open(OUTFILE, ">cog$filestr.txt");
while (<>) {
    if (/data:/) {
	$filenum--;
	$filestr = sprintf("%03d",$filenum);
	close OUTFILE;
	open(OUTFILE, ">cog$filestr.txt");
    }
    print OUTFILE $_;
}
close OUTFILE;
