<?php

$column_number1 = intval($argv[1]);
$column_number2 = intval($argv[2]);
$file1 = $argv[3];
$file2 = $argv[4];
echo "testing for column: $column_number1 vs $column_number2\n";
echo "first file: $file1\nsecond file: $file2\n";
$column_number1 = $column_number1 - 1;
$column_number2 = $column_number2 - 1;

$content1 = file_get_contents( $file1 );
$content2 = file_get_contents( $file2 );
$content1 = explode( "\n", $content1 );
$content2 = explode( "\n", $content2 );

if( count( $content1 ) != count( $content2 ) )
	echo "WARNING: file counts are not the same\n";

$maxindex = min( count($content1), count($content2 ) );

for( $i = 0; $i < $maxindex; $i++ ) {
	$c1 = explode( " ", $content1[$i] );
	$c2 = explode( " ", $content2[$i] );
	if( $c1[$column_number1] != $c2[$column_number2] )
		echo "row " . ($i+1) . ": " . $c1[$column_number1] . " : " . $c2[$column_number2] . "\n";
}


?>
