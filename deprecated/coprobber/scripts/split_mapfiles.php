#!/usr/local/bin/php
<?php

// splits the result files into files for each map
if( $argc < 2 ) {
	echo "Syntax: split_mapfiles <result_file>\n";
	exit(1);
}

// parse header of file
$data = file_get_contents( $argv[1] );
$data = explode( "\n", $data );
array_pop( $data ); // delete last row that isn't really a row anyways

// find the end of the header
$parse_header = true;
$header_index = 0;
while( $parse_header ) {
	if( strstr( $data[$header_index], "-----------------------------------" ) )
		$parse_header = false;
	$header_index++;
}

// copy the header into our formats
$header = "";
for( $i = 0; $i < $header_index; $i++ )
	$header .= $data[$i] . "\n";

$header_index++; // skip the new line after the header
$fhandle = NULL;
// for each of the other lines
for( $i = $header_index; $i < count( $data ); $i++ ) {

	// if it is a line that declares a new map
	if( strpos( $data[$i], ".map" ) ) {
		$match = basename( substr( $data[$i], 0, strpos( $data[$i], ".map" ) ) );
		// close old file
		if( $fhandle != NULL ) fclose( $fhandle );
		// open new file
		$fhandle = fopen( $argv[1] . "_" . $match . ".dat", "w" );
		// copy header information
		fwrite( $fhandle, "map file: " . $data[$i] . "\n" );
		fwrite( $fhandle, $header );

	// if it is a line with "nan"'s in it
	} else if( strstr( $data[$i], "nan" ) ) {
		echo "Warning: found \"nan\" value. Ignoring it.\n";
	// if it's a normal data line
	} else {
		fwrite( $fhandle, $data[$i] . "\n" );
	}
}
fclose( $fhandle );

?>
