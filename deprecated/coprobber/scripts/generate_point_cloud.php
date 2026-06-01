#!/usr/local/bin/php
<?php

// computes the following values for a given map and each algorithm
// averages are taken over one game simulation
//
// - suboptimality
// - average number of nodes expanded per calculation (scaled by mapsize)
// -                                  per move (scaled by mapsize)
// - average number of nodes touched  per calculation (scaled by mapsize)
// -                                  per move (scaled by mapsize)
//
// output:
// <algorithm> <suboptimality> <nE per calculation> <nT per calculation>
// <# calculations> <nE per move> <nT per move>



if( $argc < 2 ) {
	echo "Syntax: " . $argv[0] . " <algorithm_vs_algorithm> <map_stat_file(s)>\n";
	exit(1);
}


// taken from hog2/maps/bgmaps/countvertices.php
$mapsizes = array(
	"AR0082SR"=>175,
	"AR0703SR"=>354,
	"AR0408SR"=>354,
	"AR0315SR"=>531,
	"AR0527SR"=>531,
	"AR0311SR"=>558,
	"AR0415SR"=>558,
	"AR0528SR"=>562,
	"AR0335SR"=>562,
	"AR0312SR"=>562,
	"AR0710SR"=>564,
	"AR0508SR"=>567,
	"AR0416SR"=>567,
	"AR0409SR"=>576,
	"AR0407SR"=>576,
	"AR0420SR"=>689,
	"AR0531SR"=>716,
	"AR0708SR"=>716,
	"AR0325SR"=>739,
	"AR0507SR"=>739,
	"AR0503SR"=>745,
	"AR0330SR"=>811,
	"AR0501SR"=>811,
	"AR0526SR"=>833,
	"AR0417SR"=>833,
	"AR0331SR"=>833,
	"AR0319SR"=>862,
	"AR0512SR"=>896,
	"AR0713SR"=>903,
	"AR0529SR"=>903,
	"AR0313SR"=>946,
	"AR0021SR"=>946,
	"AR0332SR"=>973,
	"AR0707SR"=>974,
	"AR0706SR"=>996,
	"AR0334SR"=>996,
	"AR0413SR"=>1014,
	"AR0402SR"=>1075,
	"AR0401SR"=>1081,
	"AR0517SR"=>1083,
	"AR0316SR"=>1088,
	"AR0502SR"=>1090,
	"AR0530SR"=>1092,
	"AR0318SR"=>1092,
	"AR0514SR"=>1134,
	"AR0314SR"=>1160,
	"AR0506SR"=>1160,
	"AR0712SR"=>1163,
	"AR0022SR"=>1163,
	"AR0515SR"=>1210,
	"AR0317SR"=>1228,
	"AR0403SR"=>1337,
	"AR0418SR"=>1428,
	"AR0606SR"=>1490,
	"AR0509SR"=>1503,
	"AR0201SR"=>1557,
	"AR0704SR"=>1649,
	"AR0607SR"=>1730,
	"AR0304SR"=>1734,
	"AR0702SR"=>1768,
	"AR0302SR"=>1819,
	"AR0306SR"=>1846,
	"AR0410SR"=>1865,
	"AR0310SR"=>1884,
	"AR0020SR"=>1900,
	"AR0303SR"=>1932,
	"AR0301SR"=>1999,
	"AR0203SR"=>2037,
	"AR0709SR"=>2048,
	"AR0513SR"=>2073,
	"AR0016SR"=>2103,
	"AR0705SR"=>2248,
	"AR0041SR"=>2282,
	"AR0015SR"=>2377,
	"AR0305SR"=>2395,
	"AR0017SR"=>2401,
	"AR0045SR"=>2486,
	"AR0711SR"=>2638,
	"AR0206SR"=>2757,
	"AR0046SR"=>2761,
	"AR0600SR"=>2765,
	"AR0018SR"=>2861,
	"AR0601SR"=>2882,
	"AR0043SR"=>3036,
	"AR0069SR"=>3163,
	"AR0604SR"=>3277,
	"AR0511SR"=>3526,
	"AR0042SR"=>3584,
	"AR0605SR"=>3694,
	"AR0309SR"=>3820,
	"AR0510SR"=>4378,
	"AR0070SR"=>4606,
	"AR0308SR"=>4690,
	"AR0071SR"=>5201,
	"AR0044SR"=>5638,
	"AR0013SR"=>5672,
	"AR0072SR"=>5765,
	"AR0014SR"=>5852,
	"AR0012SR"=>6176,
	"AR0412SR"=>7627,
	"AR0516SR"=>7637,
	"AR0504SR"=>8189,
	"AR0505SR"=>8665,
	"AR0405SR"=>9348,
	"AR0202SR"=>9764,
	"AR0404SR"=>9836,
	"AR0406SR"=>11506,
	"AR0205SR"=>11540,
	"AR0603SR"=>13765,
	"AR0411SR"=>14098,
	"AR0307SR"=>14901,
	"AR0204SR"=>15899,
	"AR0701SR"=>16142,
	"AR0011SR"=>22216,
	"AR0414SR"=>22841,
	"AR0602SR"=>23314,
	"AR0400SR"=>24945,
	"AR0300SR"=>26950,
	"AR0500SR"=>29160,
	"AR0700SR"=>51586
);


// init values
$numberOfGames        = array();
$avgNodesExpandedPerC = array();
$avgNodesTouchedPerC  = array();
$avgSurvivalTimes     = array();

// store the algorithm
unset( $argv[0] );
$algorithm = $argv[1];
unset( $argv[1] );

// foreach input map
foreach( $argv as $filename ) {

	// verbose output
	//echo "including statistics for " . $filename . "\n";

	// read content data
	$data = file_get_contents( $filename );
	$data = explode( "\n", $data );
	array_pop( $data ); // delete last row that isn't actually a row

	// init
	$mapsize = 0;
	$cop_algorithms = array();
	$robber_algorithms = array();
	$robber_algorithm_names = array();

	// read header information
	$read_header = true;
	$header_count = 0;
	while( $read_header ) {

		// find map file
		if( strpos( $data[$header_count], "map file: " ) !== false ) {
			$map_file = basename( strstr( $data[$header_count], "map file: " ) );
			$map_file = substr( $map_file, 0, -4 );
			if( isset( $mapsizes[$map_file] ) )
				$mapsize = $mapsizes[$map_file];
			else {
				echo "ERROR: Map file is not registered.\n";
				exit(1);
			}
		}
		// find cop algorithms
		else if( strpos( $data[$header_count], "cop algorithms: " ) !== false ) {
			$temp = explode( " ", $data[$header_count] );
			for( $i = 2; $i < count($temp); $i++ ) {
				if( $temp[$i] != "" )
					$cop_algorithms[] = $temp[$i];
			}
		}
		// find names of robber algorithms
		else if( strpos( $data[$header_count], "robber algorithms:" ) !== false ) {
			$header_count++;
			while( strpos( $data[$header_count], "  " ) !== false ) {
				$robber_algorithm_names[] = substr( $data[$header_count], 2 );
				$header_count++; // advance one row
			}
		}
		else if( strstr( $data[$header_count], "------------------------------" ) )
			$read_header = false;

		$header_count++;
	}

	
	// if mapsize could not be determined
	if( $mapsize == 0 ) {
		echo "ERROR: map file parameter not submitted.\n";
		exit(1);
	}


	// parse robber algorithm names
	foreach( $robber_algorithm_names as $ra ) {
		if( $ra == "optimal( )" || $ra == "cover( )" || $ra == "greedy( )" )
			$robber_algorithms[] = substr( $ra, 0, -3 );
		elseif( preg_match( "/^cover2\( \(double\)([\d\.-]+) \)/", $ra, $match ) )
			$robber_algorithms[] = "cover2(" . $match[1] . ")";
		elseif( preg_match( "/^minimax\( \(double\)([\d\.-]+) \)/", $ra, $match ) )
			$robber_algorithms[] = "minimax(" . $match[1] . ")";
		elseif( preg_match( "/^dam\( \(double\)([\d\.-]+) \(double\)([\d\.-]+) \)/", $ra, $match ) )
			$robber_algorithms[] = "dam(" . $match[1] . "," . $match[2] . ")";
		elseif( preg_match( "/^idam\( \(int\)(\d+) \(int\)(\d+) \(double\)([\d\.\-]+) \(double\)([\d\.\-]+) \)/", $ra, $match ) ) {
			for( $i = intval( $match[1] ); $i <= intval( $match[2] ); $i++ )
				$robber_algorithms[] = "idam(" . $i . "," . $match[3] . "," . $match[4] . ")";
		}
		elseif( preg_match( "/^idam2\( \(double\)([\d\.-]+) \(double\)([\d\.-]+) \)/", $ra, $match ) )
			$robber_algorithms[] = "idam2(" . $match[1] . "," . $match[2] . ")";
		elseif( preg_match( "/^randombeacons\( \(int\)(\d+) \(int\)(\d+) \(int\)(\d+) \)/", $ra, $match ) ) {
			for( $i = intval( $match[2] ); $i <= intval( $match[3] ); $i++ )
				$robber_algorithms[] = "randombeacons(" . $match[1] . "," . $i . ")";
		}
		elseif( preg_match( "/^trailmax\( \(int\)(\d+) \(int\)(\d+) \)/", $ra, $match ) ) {
			for( $i = intval( $match[1] ); $i <= intval( $match[2] ); $i++ )
				$robber_algorithms[] = "trailmax(" . $i . ")";
		}
		elseif( preg_match( "/^datrailmax\( \(int\)(\d+) \(int\)(\d+) \(int\)(\d+) \)/", $ra, $match ) ) {
			for( $i = intval( $match[1] ); $i <= intval( $match[2] ); $i++ )
				$robber_algorithms[] = "datrailmax(" . $i . "," . $match[3] . ")";
		}
		else {
			echo "ERROR: unsupported robber algorithm detected: $ra.\n";
			exit( 1 );
		}
	}

	// find out whether the optimal game value is available
	$optimal_values_available = false;
	foreach( $cop_algorithms as $ca ) {
		if( $ca == "optimal" ) $optimal_values_available = true;
		foreach( $robber_algorithms as $ra ) {
			if( $ra == "optimal" ) { $optimal_values_available = true; break; }
		}
		if( $optimal_values_available ) break;
	}

	// the optimal values for the games have not been computed
	if( !$optimal_values_available ) {
		echo "ERROR: optimal values have not been computed in file $filename.\n";
		exit(1);
	}

	// now parse all the rows of the file
	for( $row = $header_count; $row < count( $data ); $row++ ) {

		$values = explode( " ", $data[$row] );

		// $values[0] => robber position
		// $values[1] => cop position
		// $values[2] => optimal game value (if optimum has not been computed we exited the script before)
		$optimal_value = ceil($values[2]-1)/2;
		$row_index = 3;

		foreach( $cop_algorithms as $ca ) {
			foreach( $robber_algorithms as $ra ) {
				if( $ca == "optimal" && $ra == "optimal" ) continue;

				$index = $ra . "_vs_" . $ca;

				// we are only interested in the pointclouds of the submitted algorithm
				if( $index != $algorithm ) {
					if( $ra == "optimal" ) $row_index +=1;
					else $row_index += 6;
					continue;
				}
				
				if( $ra == "optimal" ) {
					// in case we have the optimal robber we only measure the suboptimality of the cop
					echo ((double)$values[$row_index]/(double)$optimal_value);
					$row_index += 1;
				} else {

					$nodesExpanded = $values[$row_index+1] * $values[$row_index+4];
					$nodesTouched  = $values[$row_index+1] * $values[$row_index+5];
					$numberCalls   = $values[$row_index+1];
					$numberTurns   = ceil( ($values[$row_index]-1)/2 );

					// survival time
					echo ((double)$numberTurns  /(double)$optimal_value) . " ";
					// nodes expanded per call
					echo ((double)$nodesExpanded/(double)$numberCalls/$mapsize) . " ";
					// nodes touched per call
					echo ((double)$nodesTouched /(double)$numberCalls/$mapsize) . " ";
					// nodes expanded per turn
					echo ((double)$nodesExpanded/(double)$numberTurns/$mapsize) . " ";
					// nodes touched per turn
					echo ((double)$nodesTouched /(double)$numberTurns/$mapsize) . "\n";
					$row_index += 6;
				}
			}
		}
	}

}

?>
