// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>

struct state
{
	double yaw, pitch, lastX, lastY;
	bool ml;
	std::vector<rs::stream> tex_streams;
	float depth_scale;
	rs::extrinsics extrin;
	rs::intrinsics depth_intrin;
	rs::intrinsics tex_intrin;
	bool identical;
	int index;
	rs::device * dev;
};

int INPUT_WIDTH 	= 320;
int INPUT_HEIGHT 	= 240;
int FRAMERATE 		= 60;


static rs::context ctx;
static state app_state;



state *initialize_app_state( )
{
	if( ctx.get_device_count( ) > 0 )
	{
		static rs::device & dev = *ctx.get_device( 0 );


		dev.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
		dev.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
		dev.start( );

		state initState;
		initState.yaw 			= 0;
		initState.pitch 		= 0;
		initState.lastX 		= 0;
		initState.lastY			= 0;
		initState.ml			= false;
		initState.tex_streams	= { rs::stream::color, rs::stream::depth, rs::stream::infrared };
		initState.depth_scale	= dev.get_depth_scale( );
		initState.extrin		= dev.get_extrinsics( rs::stream::depth, rs::stream::color );
		initState.depth_intrin	= dev.get_stream_intrinsics( rs::stream::depth );
		initState.tex_intrin	= dev.get_stream_intrinsics( rs::stream::depth );
		initState.identical		= false;
		initState.index			= 0;
		initState.dev			= &dev;
/*
		state initState =
			{
				0, 0, 0, 0,
				false,
				{
					rs::stream::color, rs::stream::depth, rs::stream::infrared }, dev.get_depth_scale(),


			dev.get_extrinsics(rs::stream::depth, rs::stream::color), dev.get_stream_intrinsics(rs::stream::depth),
			dev.get_stream_intrinsics(rs::stream::depth), 0, 0, &dev };
*/
		app_state = initState;

		return &app_state;
	}
	return 0;
}



bool app_next_frame( ) //(int &xInOut, int &yInOut, int &zInOut)
{
	rs::device & dev = *app_state.dev;

	if( dev.is_streaming( ) )
		dev.wait_for_frames( );

	const rs::stream tex_stream = app_state.tex_streams[ app_state.index ];
	app_state.depth_scale 		= dev.get_depth_scale( );
	app_state.extrin 			= dev.get_extrinsics( rs::stream::depth, tex_stream );
	app_state.depth_intrin 		= dev.get_stream_intrinsics( rs::stream::depth );
	app_state.tex_intrin 		= dev.get_stream_intrinsics( tex_stream );
	app_state.identical 		= app_state.depth_intrin == app_state.tex_intrin && app_state.extrin.is_identity( );

	// setup the OpenCV Mat structures
	cv::Mat depth16( app_state.depth_intrin.height, app_state.depth_intrin.width, CV_16U,
										(uchar *)dev.get_frame_data( rs::stream::depth ) );

	rs::intrinsics 	color_intrin = dev.get_stream_intrinsics( rs::stream::color );
	cv::Mat 		rgb( color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev.get_frame_data( rs::stream::color ) );

	cv::Mat depth8u = depth16;// < 800;

	depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );

	imshow( "depth8u", depth8u );
	cvWaitKey( 1 );

	cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
	imshow( "rgb", rgb );
	cvWaitKey( 1 );

	return true;
}




//int main( int argc, char * argv[] ) try
int main( ) try
{
	rs::log_to_console( rs::log_severity::warn );

	state *app_state = initialize_app_state( );

	if (app_state == 0)
	{
		std::cout << "Unable to locate a camera" << std::endl;
		rs::log_to_console( rs::log_severity::fatal );
		return EXIT_FAILURE;
	}

	while( true )
	{
		app_next_frame( );//(handPoint.x, handPoint.y, z);
	}

	return EXIT_SUCCESS;

}
catch( const rs::error & e )
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch( const std::exception & e )
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
