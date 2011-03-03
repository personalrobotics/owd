$old=0; while (<>) {if (/(\d+),(\d+).*G04 GET 048/) {$new=$1*1000000+$2; $delta=$new-$old; if ($old > 0) {print "$new $delta\n";} $old=$new;}}
