<?php

$data = file_get_contents( "set2_1.dat" );
$data = explode( "\n", $data );
array_pop( $data );

foreach( $data as $row ) {
	$row = explode( " ", $row );
	$cover     = (double)$row[8]; // 4
	$minimax   = (double)$row[14]; // 10
	$dam       = (double)$row[20]; // 16
	$heuristic = (double)$row[26]; // 22
	// 6 * 20 + 22 = 142
	$pathmax   = (double)$row[142] * (double)$row[146] / (double)($row[141]-1)*2;
	// 6 * 20 + 142 = 262
	$dapathmax = (double)$row[262] * (double)$row[266] / (double)($row[261]-1)*2;
	// 6 * 20 + 262 = 382
	$beacon    = (double)$row[382] * (double)$row[386] / (double)($row[381]-1)*2;
	echo $cover . " " . $minimax . " " . $dam . " " . $heuristic . " " . $pathmax . " " . $dapathmax . " " . $beacon . "\n";
}


?>
