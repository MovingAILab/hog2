<?php

$list = scandir( "." );

$countlist = array();

foreach( $list as $filename ) {
	if( substr( $filename, 0, 2 ) == "AR" ) {
		$content = file_get_contents( $filename );
		str_replace( ".", "", $content, $nodecount );
		$countlist[$filename] = $nodecount;
	}
}

asort( $countlist );
foreach( $countlist as $f => $c ) {
	echo $f . " : " . $c . "\n";
}

?>
