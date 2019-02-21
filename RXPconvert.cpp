// $Id: pointcloudcpp.cpp 685 2012-06-18 07:47:18Z RS $

// (c) Riegl 2008
// pointcloudcpp.cpp - Example for reading pointcloud data
// from Riegl's *.rxp format.
//
// This example uses the RiVLib as a statically linked C++ library.
//
// Compile instructions:
//   Please read the instructions in CmakeLists.txt
//
// Usage instructions:
// Invoke the program as:
//   pointcloudcpp <uri>
//   where uri is e.g. 'file:../scan.rxp' when reading from a file or
//   'rdtp://ip-addr/current' when reading real-time data from the scanner
//   (replace ip-addr with the ip-addr of your scanner).
// The program will read the pointcloud data, filter out single and first targets
// and print them out to the console in ASCII.
//

#include <riegl/scanlib.hpp>

#include <iostream>
#include <fstream>
#include <exception>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <memory>
#include <string.h>

using namespace scanlib;
using namespace std;
//using namespace std::tr1;

// The import class is derived from pointcloud class, which assembles the
// scanner data into distinct targets and computes x,y,z pointcloud data.
// The pointcloud class and its base class have a huge number of overridables
// that give access to all of the scanners data, e.g. "gps" or "housekeeping"
// data. The pointcloud class also has the necessary logic to align the
// data to gps information (if embedded in the rxp stream) and will return
// timestamps in the domain of the gps.

class importer
    : public pointcloud
{
    ostream& o;
    unsigned long line;
	bool isGPS;
	vector<char> outvars;
	scanlib::target Target0s[1];
	std::string points_to_print;
	int numpointswritten;
	bool isbinary;
public:
    importer(ostream& o_, bool isGPS, vector<char>& outvar, string &points2print, bool isBin)
        : pointcloud(isGPS) // set this to true if you need gps aligned timing
        , o(o_)
        , line(0)
		, outvars(outvar)
		, points_to_print(points2print)
		, numpointswritten(0)
		, isbinary(isBin)
    {
        o.precision(10);
    }

protected:

    // overridden from pointcloud class
    void on_shot_end()
	{
		//build
		//id
		//num_facets;
		//serial
		//type_id
		
		//allshots
		//allpoints
		//first returns

//		double phi=180/pi*atan2(beam_direction[1],beam_direction[0]);
//		double theta=180/pi*atan2(beam_direction[2],sqrt((beam_direction[0]*beam_direction[0])+(beam_direction[1]*beam_direction[1])));
		int numtargetstoprocess=target_count;
		scanlib::target Target0s[1];

		if (target_count==0 && strcmp(points_to_print.c_str(),"allshots")==0)
		{
			targets[0]=Target0s[0];
			numtargetstoprocess=1;
		}
		for (int ii=0; ii<numtargetstoprocess; ii++) {
			if (strcmp(points_to_print.c_str(),"first")==0 && ii>0) {
				continue;
			}
			if (strcmp(points_to_print.c_str(),"last")==0 && ii!=target_count-1) {
				continue;
			}
			for(int i=0;i<outvars.size();i++){
			
				if (!isbinary){
					if (i>0) o << "," ;			

					switch(outvars[i]){
					case 'a': o << targets[ii].amplitude; break;// dB
					case 'b': o << targets[ii].background_radiation; break; //
					case 'd': o << targets[ii].deviation; break; // bigger=more distortion
					case 'R': o << targets[ii].echo_range; break; // Range from Mirror (NOT PHASE CENTER) !=sqrt(x^2 y^2 z^2)
					case 'f': o << facet; break; // Which Mirror Face
					case 'h': o << targets[ii].is_high_power; break; // 
					case 'p': o << targets[ii].is_pps_locked; break; // 
					case 'e': o << targets[ii].is_rising_edge; break; // 
					case 's': o << targets[ii].is_sw; break; // 
					case 'r': o << targets[ii].reflectance; break; // dB
					case 't': o << time; break; // time with reference determined by sync_to_gps... can't return both
					case 'x': o << targets[ii].vertex[0]; break;// X
					case 'y': o << targets[ii].vertex[1]; break;// Y
					case 'z': o << targets[ii].vertex[2]; break; // Z
					case '0': o << beam_origin[0]; break; // Beam Origin x
					case '1': o << beam_origin[1]; break; // Beam Origin y
					case '2': o << beam_origin[2]; break; // Beam Origin z
					case '3': o << beam_direction[0]; break; // Beam Direction x
					case '4': o << beam_direction[1]; break; // Beam Direction y
					case '5': o << beam_direction[2]; break; // Beam Direction z
					case 'n': 
						if (target_count==0) o << 0;  // return number
						else o << ii+1;// return number
						break;
					case 'N': o << target_count; break; // number of returns for given pulse
					}
					if (i==outvars.size()-1){
						o << endl;
					}
				}
				else
				{
					double val=0;
				switch(outvars[i]){
				case 'a':  val= targets[ii].amplitude; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break;// dB
				case 'b':  val= targets[ii].background_radiation; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; //
				case 'd':  val= targets[ii].deviation; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // bigger=more distortion
				case 'R':  val= targets[ii].echo_range; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Range from Mirror (NOT PHASE CENTER) !=sqrt(x^2 y^2 z^2)
				case 'f':  val= facet; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Which Mirror Face
				case 'h':  val= targets[ii].is_high_power; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // 
				case 'p':  val= targets[ii].is_pps_locked; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // 
				case 'e':  val= targets[ii].is_rising_edge; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // 
				case 's':  val= targets[ii].is_sw; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // 
				case 'r':  val= targets[ii].reflectance; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // dB
				case 't':  val= time; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // time with reference determined by sync_to_gps... can't return both
				case 'x':  val= targets[ii].vertex[0]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break;// X
				case 'y':  val= targets[ii].vertex[1]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break;// Y
				case 'z':  val= targets[ii].vertex[2]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Z
				case '0':  val= beam_origin[0]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Origin x
				case '1':  val= beam_origin[1]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Origin y
				case '2':  val= beam_origin[2]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Origin z
				case '3':  val= beam_direction[0]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Direction x
				case '4':  val= beam_direction[1]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Direction y
				case '5':  val= beam_direction[2]; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // Beam Direction z
				case 'n': 
					if (target_count==0)  val= 0;  // return number
					else  val= ii+1;// return number
					o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break;
				case 'N':  val= target_count; o.write( reinterpret_cast <const char*> (&val), sizeof( val ) );break; // number of returns for given pulse
				}
				}
		}

			numpointswritten++;
			if (numpointswritten % 50000 == 0)
				cout << "Number of Points Written :" << numpointswritten << endl;
				
		}
	}
/*	
	void on_echo_transformed(echo_type echo)
    {
        // here we select which target types we are interested in
        if ( pointcloud::last == echo || pointcloud::single == echo) {
            // targets is a member std::vector that contains all
            // echoes seen so far, i.e. the current echo is always
            // indexed by target_count-1.
            target& t(targets[target_count-1]);
			beam_direction;
            // transform to polar coordinates
            double range = std::sqrt(
                t.vertex[0]*t.vertex[0]
                + t.vertex[1]*t.vertex[1]
                + t.vertex[2]*t.vertex[2]
            );
            if (range > numeric_limits<double>::epsilon()) {
                double phi = atan2(t.vertex[1],t.vertex[0]);
                phi = ((phi<0.0)?(phi+2.0*pi):phi);
                double theta = std::acos(t.vertex[2]/range);
                t.vertex[0] = static_cast<float>(range);
                t.vertex[1] = static_cast<float>((360.0/(2.0*pi))*theta);
                t.vertex[2] = static_cast<float>((360.0/(2.0*pi))*phi);
            }
            else
                t.vertex[0] = t.vertex[1] = t.vertex[2] = 0.0;


			
            // print out the result
    //        o << t.vertex[0] << ", " << t.vertex[1] << ", " << t.vertex[2] << ", " << t.time << endl;

        }
		else if (pointcloud::echo_type::none==echo){
			o << "0,0,0" << endl;
		}
    }
	*/
    // overridden from basic_packets
    // this function gets called when a the scanner emits a notification
    // about an exceptional state.
    
	/*
	void on_unsolicited_message(const unsolicited_message<iterator_type>& arg) {
        pointcloud::on_unsolicited_message(arg);
        // in this example we just print a warning to stderr
 //       cerr << "MESSAGE: " << arg.message << endl;
        // the following line would put out the entire content of the packet
        // converted to ASCII format:
        // cerr << arg << endl;
    }
    void on_line_start_up(const line_start_up<iterator_type>& arg) {
        pointcloud::on_line_start_up(arg);
//        o << "line: " << ++line << endl;
    }
    void on_line_start_dn(const line_start_dn<iterator_type>& arg) {
        pointcloud::on_line_start_dn(arg);
//        o << "line: " << ++line << endl;
    }
	void on_scanner_pose(const scanner_pose<iterator_type>& arg) {
		pointcloud::on_scanner_pose(arg);
//		o << "scanner: " << endl << arg.roll << endl << arg.pitch << endl << arg.yaw << endl;
	}
	void on_hk_pwr(const hk_pwr<iterator_type>& arg) {
		pointcloud::on_hk_pwr(arg);
		o << arg << endl;
	}
	
    void on_id(const package_id& arg){
		pointcloud::on_id(arg);
	o << "on_id: " << arg << endl;
	}
	 
    //!\param arg  IMU raw data
     void on_IMU_data(const IMU_data<iterator_type>& arg){
		pointcloud::on_IMU_data(arg);
		o << "on_IMU_data " << endl << arg << endl;
	}
    //!\param arg  environmental information
     void on_atmosphere(const atmosphere<iterator_type>& arg){
		pointcloud::on_atmosphere(arg);
		o << "on_atmosphere " << endl << arg << endl;
	}
    //!\param arg  extended environmental information
     void on_atmosphere_1(const atmosphere_1<iterator_type>& arg){
		pointcloud::on_atmosphere_1(arg);
		o << "on_atmosphere_1 " << endl << arg << endl;
	}
    //!\param arg  extended environmental information
     void on_atmosphere_2(const atmosphere_2<iterator_type>& arg){
		pointcloud::on_atmosphere_2(arg);
		o << "on_atmosphere_2 " << endl << arg << endl;
	}
    //!\param arg  extended environmental information
     void on_atmosphere_3(const atmosphere_3<iterator_type>& arg){
		pointcloud::on_atmosphere_3(arg);
		o << "on_atmosphere_3 " << endl << arg << endl;
	}
    //!\param arg  laser beam description
     void on_beam_geometry(const beam_geometry<iterator_type>& arg){
		pointcloud::on_beam_geometry(arg);
		o << "on_beam_geometry " << endl << arg << endl;
	}
    //!\param arg  modelling biaxial shift of cog of range measurement in the near range
     void on_biaxial_geometry(const biaxial_geometry<iterator_type>& arg){
		pointcloud::on_biaxial_geometry(arg);
		o << "on_biaxial_geometry " << endl << arg << endl;
	}
    //!\param arg  external synchronization input
     void on_counter_sync(const counter_sync<iterator_type>& arg){
		pointcloud::on_counter_sync(arg);
		o << "on_counter_sync " << endl << arg << endl;
	}
    //!\param arg  geometrical parameters of external devices
     void on_device_mounting(const device_mounting<iterator_type>& arg){
		pointcloud::on_device_mounting(arg);
		o << "on_device_mounting " << endl << arg << endl;
	}
    //!\param arg  extents of various data fields
     void on_extents(const extents<iterator_type>& arg){
		pointcloud::on_extents(arg);
		o << "on_extents " << endl << arg << endl;
	}
    //!\param arg  start of a scan frame in down direction.
     void on_frame_start_dn(const frame_start_dn<iterator_type>& arg){
		pointcloud::on_frame_start_dn(arg);
		o << "on_frame_start_dn " << endl << arg << endl;
	}
    //!\param arg  Start of a scan frame in up direction.
     void on_frame_start_up(const frame_start_up<iterator_type>& arg){
		pointcloud::on_frame_start_up(arg);
		o << "on_frame_start_up " << endl << arg << endl;
	}
    //!\param arg  end of a scan frame
     void on_frame_stop(const frame_stop<iterator_type>& arg){
		pointcloud::on_frame_stop(arg);
		o << "on_frame_stop " << endl << arg << endl;
	}
    //!\param arg  The mandatory header package.
     void on_header(const header<iterator_type>& arg){
		pointcloud::on_header(arg);
		o << "on_header " << endl << arg << endl;
	}
    //!\param arg  Extension header
     void on_header_ext(const header_ext<iterator_type>& arg){
		pointcloud::on_header_ext(arg);
		o << "on_header_ext " << endl << arg << endl;
	}
    //!\param arg  power supply unit (24 Bytes)
     void on_hk_bat(const hk_bat<iterator_type>& arg){
		pointcloud::on_hk_bat(arg);
		o << "on_hk_bat " << endl << arg << endl;
	}
    //!\param arg  power supply unit (24 Bytes)
     void on_hk_bat_1(const hk_bat_1<iterator_type>& arg){
		pointcloud::on_hk_bat_1(arg);
		o << "on_hk_bat_1 " << endl << arg << endl;
	}
    //!\param arg  power supply unit (24 Bytes)
     void on_hk_bat_2(const hk_bat_2<iterator_type>& arg){
		pointcloud::on_hk_bat_2(arg);
		o << "on_hk_bat_2 " << endl << arg << endl;
	}
    //!\param arg  control unit, i.e., mother board, including storage levels, info on attached equipment (60 Bytes)
     void on_hk_ctr(const hk_ctr<iterator_type>& arg){
		pointcloud::on_hk_ctr(arg);
		o << "on_hk_ctr " << endl << arg << endl;
	}
    //!\param arg  control unit, i.e., mother board, including storage levels, info on attached equipment (60 Bytes)
     void on_hk_ctr_1(const hk_ctr_1<iterator_type>& arg){
		pointcloud::on_hk_ctr_1(arg);
		o << "on_hk_ctr_1 " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps(const hk_gps<iterator_type>& arg){
		pointcloud::on_hk_gps(arg);
		o << "on_hk_gps " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps_hr(const hk_gps_hr<iterator_type>& arg){
		pointcloud::on_hk_gps_hr(arg);
		o << "on_hk_gps_hr " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps_ts(const hk_gps_ts<iterator_type>& arg){
		pointcloud::on_hk_gps_ts(arg);
		o << "on_hk_gps_ts " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps_ts_status(const hk_gps_ts_status<iterator_type>& arg){
		pointcloud::on_hk_gps_ts_status(arg);
		o << "on_hk_gps_ts_status " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps_ts_status_dop(const hk_gps_ts_status_dop<iterator_type>& arg){
		pointcloud::on_hk_gps_ts_status_dop(arg);
		o << "on_hk_gps_ts_status_dop " << endl << arg << endl;
	}
    //!\param arg  GPS data
     void on_hk_gps_ts_status_dop_ucs(const hk_gps_ts_status_dop_ucs<iterator_type>& arg){
		pointcloud::on_hk_gps_ts_status_dop_ucs(arg);
		o << "on_hk_gps_ts_status_dop_ucs " << endl << arg << endl;
	}
    //!\param arg  inclination sensor
     void on_hk_incl(const hk_incl<iterator_type>& arg){
		pointcloud::on_hk_incl(arg);
		o << "on_hk_incl " << endl << arg << endl;
	}
    //!\param arg  inclination sensor
     void on_hk_incl_4axes(const hk_incl_4axes<iterator_type>& arg){
		pointcloud::on_hk_incl_4axes(arg);
		o << "on_hk_incl_4axes " << endl << arg << endl;
	}
    //!\param arg  protective housing data
     void on_hk_ph_data(const hk_ph_data<iterator_type>& arg){
		pointcloud::on_hk_ph_data(arg);
		o << "on_hk_ph_data " << endl << arg << endl;
	}
    //!\param arg  protective housing data
     void on_hk_ph_data_1(const hk_ph_data_1<iterator_type>& arg){
		pointcloud::on_hk_ph_data_1(arg);
		o << "on_hk_ph_data_1 " << endl << arg << endl;
	}
    //!\param arg  protective housing units
     void on_hk_ph_units(const hk_ph_units<iterator_type>& arg){
		pointcloud::on_hk_ph_units(arg);
		o << "on_hk_ph_units " << endl << arg << endl;
	}
    //!\param arg  protective housing units
     void on_hk_ph_units_1(const hk_ph_units_1<iterator_type>& arg){
		pointcloud::on_hk_ph_units_1(arg);
		o << "on_hk_ph_units_1 " << endl << arg << endl;
	}
    //!\param arg  power supply unit (24 Bytes)
     void on_hk_pwr(const hk_pwr<iterator_type>& arg){
		pointcloud::on_hk_pwr(arg);
		o << "on_hk_pwr " << endl << arg << endl;
	}
    //!\param arg  power supply unit (24 Bytes)
     void on_hk_pwr_1(const hk_pwr_1<iterator_type>& arg){
		pointcloud::on_hk_pwr_1(arg);
		o << "on_hk_pwr_1 " << endl << arg << endl;
	}
    //!\param arg  built in real time clock of scanning device, local time
     void on_hk_rtc(const hk_rtc<iterator_type>& arg){
		pointcloud::on_hk_rtc(arg);
		o << "on_hk_rtc " << endl << arg << endl;
	}
    //!\param arg  real time clock, local time (10 Bytes)
     void on_hk_rtc_sys(const hk_rtc_sys<iterator_type>& arg){
		pointcloud::on_hk_rtc_sys(arg);
		o << "on_hk_rtc_sys " << endl << arg << endl;
	}
    //!\param arg  wyler inclination sensor
     void on_inclination_wyler(const inclination_wyler<iterator_type>& arg){
		pointcloud::on_inclination_wyler(arg);
		o << "on_inclination_wyler " << endl << arg << endl;
	}
    //!\param arg  start of a line scan in down direction
     void on_line_start_dn(const line_start_dn<iterator_type>& arg){
		pointcloud::on_line_start_dn(arg);
		o << "on_line_start_dn " << endl << arg << endl;
	}
    //!\param arg  start of a line scan in up direction
     void on_line_start_up(const line_start_up<iterator_type>& arg){
		pointcloud::on_line_start_up(arg);
		o << "on_line_start_up " << endl << arg << endl;
	}
    //!\param arg  end of line scan
     void on_line_stop(const line_stop<iterator_type>& arg){
		pointcloud::on_line_stop(arg);
		o << "on_line_stop " << endl << arg << endl;
	}
    //!\param arg  measurement has started
     void on_meas_start(const meas_start<iterator_type>& arg){
		pointcloud::on_meas_start(arg);
		o << "on_meas_start " << endl << arg << endl;
	}
    //!\param arg  measurement has stopped
     void on_meas_stop(const meas_stop<iterator_type>& arg){
		pointcloud::on_meas_stop(arg);
		o << "on_meas_stop " << endl << arg << endl;
	}
    //!\param arg  current setting of subdivider for monitoring data stream
     void on_monitoring_info(const monitoring_info<iterator_type>& arg){
		pointcloud::on_monitoring_info(arg);
		o << "on_monitoring_info " << endl << arg << endl;
	}
    //!\param arg  default parameters for MTA processing
     void on_mta_settings(const mta_settings<iterator_type>& arg){
		pointcloud::on_mta_settings(arg);
		o << "on_mta_settings " << endl << arg << endl;
	}
    //!\param arg  default parameters for MTA processing
     void on_mta_settings_1(const mta_settings_1<iterator_type>& arg){
		pointcloud::on_mta_settings_1(arg);
		o << "on_mta_settings_1 " << endl << arg << endl;
	}
    //!\param arg  pulse per second, external time synchronisation
     void on_pps_sync(const pps_sync<iterator_type>& arg){
		pointcloud::on_pps_sync(arg);
		o << "on_pps_sync " << endl << arg << endl;
	}
    //!\param arg  pulse per second, external time synchronisation
     void on_pps_sync_ext(const pps_sync_ext<iterator_type>& arg){
		pointcloud::on_pps_sync_ext(arg);
		o << "on_pps_sync_ext " << endl << arg << endl;
	}
    //!\param arg  pulse per second, external time synchronisation
     void on_pps_sync_hr(const pps_sync_hr<iterator_type>& arg){
		pointcloud::on_pps_sync_hr(arg);
		o << "on_pps_sync_hr " << endl << arg << endl;
	}
    //!\param arg  pulse per second, external time synchronisation
     void on_pps_sync_hr_ext(const pps_sync_hr_ext<iterator_type>& arg){
		pointcloud::on_pps_sync_hr_ext(arg);
		o << "on_pps_sync_hr_ext " << endl << arg << endl;
	}
    //!\param arg  scan pattern description
     void on_scan_rect_fov(const scan_rect_fov<iterator_type>& arg){
		pointcloud::on_scan_rect_fov(arg);
		o << "on_scan_rect_fov " << endl << arg << endl;
	}
    //!\param arg  scanner pose (position and orientation
     void on_scanner_pose(const scanner_pose<iterator_type>& arg){
		pointcloud::on_scanner_pose(arg);
		o << "on_scanner_pose " << endl << arg << endl;
	}
    //!\param arg  scanner pose (position and orientation
     void on_scanner_pose_hr(const scanner_pose_hr<iterator_type>& arg){
		pointcloud::on_scanner_pose_hr(arg);
		o << "on_scanner_pose_hr " << endl << arg << endl;
	}
    //!\param arg  scanner pose (position and orientation
     void on_scanner_pose_hr_1(const scanner_pose_hr_1<iterator_type>& arg){
		pointcloud::on_scanner_pose_hr_1(arg);
		o << "on_scanner_pose_hr_1 " << endl << arg << endl;
	}
    //!\param arg  scanner pose in user coordinate system (ucs)
     void on_scanner_pose_ucs(const scanner_pose_ucs<iterator_type>& arg){
		pointcloud::on_scanner_pose_ucs(arg);
		o << "on_scanner_pose_ucs " << endl << arg << endl;
	}
     void on_scanner_pose_ucs_1(const scanner_pose_ucs_1<iterator_type>& arg){
		pointcloud::on_scanner_pose_ucs_1(arg);
		o << "on_scanner_pose_ucs_1 " << endl << arg << endl;
	}
    //!\param arg  units information
     void on_units(const units<iterator_type>& arg){
		pointcloud::on_units(arg);
		o << "on_units " << endl << arg << endl;
	}
    //!\param arg  units information
     void on_units_1(const units_1<iterator_type>& arg){
		pointcloud::on_units_1(arg);
		o << "on_units_1 " << endl << arg << endl;
	}
    //!\param arg  units information
     void on_units_2(const units_2<iterator_type>& arg){
		pointcloud::on_units_2(arg);
		o << "on_units_2 " << endl << arg << endl;
	}
    //!\param arg  units information
     void on_units_IMU(const units_IMU<iterator_type>& arg){
		pointcloud::on_units_IMU(arg);
		o << "on_units_IMU " << endl << arg << endl;
	}
    //!\param arg  spontaneous information and error messages
     void on_unsolicited_message(const unsolicited_message<iterator_type>& arg){
		pointcloud::on_unsolicited_message(arg);
		o << "on_unsolicited_message " << endl << arg << endl;
	}
    //!\param arg  spontaneous information and error messages
     void on_unsolicited_message_1(const unsolicited_message_1<iterator_type>& arg){
		pointcloud::on_unsolicited_message_1(arg);
		o << "on_unsolicited_message_1 " << endl << arg << endl;
	}
    //!\param arg  empty helper package
     void on_void_data(const void_data<iterator_type>& arg){
		pointcloud::on_void_data(arg);
		o << "on_void_data " << endl << arg << endl;
	}

	void on_laser_shot(const laser_shot<iterator_type>& arg){
		pointcloud::on_laser_shot(arg);
		o << "on_laser_shot " << endl << arg << endl;
	}
	*/


};
void showHelp()
{
	cout << "*** HELP *** " << endl << endl;
	cout << "example:" << endl << "RXPconvert.exe Scan1.rxp Testfile.txt allpoints x y z a tgps" << endl << endl;
	cout << "\t Argument 1: 'Scan1.rxp' is the RXP file to be converted" << endl;
	cout << "\t Argument 2: 'Testfile.txt' is the file that is being written to" << endl << endl;
	cout << "\t\t Option Filename Extensions are: " << endl;
	cout << "\t\t   '.txt' makes a comma delimited file with headers" << endl;
	cout << "\t\t   '.bin' makes a binary file of all 8 byte doubles *fastest*" << endl;
	cout << "\t\t   '.csv/other' makes a comma delimited file without headers" << endl << endl;
	cout << "\t Argument 3: 'allpoints' is which points to return" << endl << endl;
	cout << "\t\t Option Arguments are: " << endl;
	cout << "\t\t   'allshots' returns a point even when no shots returned" << endl;
	cout << "\t\t\t useful to return the Beam Direction with 'allshots'" << endl;
	cout << "\t\t   'allpoints' returns every point that was returned" << endl;
	cout << "\t\t   'first' returns only the first points returned" << endl;
	cout << "\t\t   'last' returns only the last points returned" << endl; 
	cout << "\t\t\t ** if only one point is returned it is will be written\n\t\t\t ** with both the 'first' and 'last' tag" << endl << endl;
	cout << "\t Arguments 4+: 'x y z a tgps' is the data written to the text file" << endl << endl;
	cout << "\t\t Optional Arguments are: " << endl;
	cout << "\t\t  'a': amplitude (dB)" << endl;
	cout << "\t\t  'b': background_radiation" << endl;
	cout << "\t\t  'd': deviation" << endl;
	cout << "\t\t  'R': echo_range" << endl;
	cout << "\t\t  'f': facet number" << endl;
	cout << "\t\t  'h': is_high_power" << endl;
	cout << "\t\t  'p': is_pps_locked" << endl;
	cout << "\t\t  'e': is_rising_edge" << endl;
	cout << "\t\t  's': is_sw" << endl;
	cout << "\t\t  'r': reflectance (dB)" << endl;
	cout << "\t\t  'tgps': GPS time (can't be used with 'tint')" << endl;
	cout << "\t\t  'tint': Internal time (can't be used with 'tgps')" << endl;
	cout << "\t\t  'x': x position of point in the SOCS (m)" << endl;
	cout << "\t\t  'y': y position of point in the SOCS (m)" << endl;
	cout << "\t\t  'z': z position of point in the SOCS (m)" << endl;
	cout << "\t\t  '0': beam_origin[0] (x distance from phase center) (m)" << endl;
	cout << "\t\t  '1': beam_origin[1] (y distance from phase center) (m)" << endl;
	cout << "\t\t  '2': beam_origin[2] (z distance from phase center) (m)" << endl;
	cout << "\t\t  '3': beam_direction[0] (x component of unit direction vector)" << endl;
	cout << "\t\t  '4': beam_direction[1] (y component of unit direction vector)" << endl;
	cout << "\t\t  '5': beam_direction[2] (z component of unit direction vector)" << endl;
	cout << "\t\t  'n': number of return" << endl;
	cout << "\t\t  'N': number of returns for given pulse" << endl;

	cout << endl << "\t\t *NOTE.  if using 'beam_direction' and 'range', you must take\n\t\t * into account the 'beam_origin' offset to get to the SOCS" << endl;
	cout << endl << "\t\t *SOCS = Scanners Own Coordinate System" << endl;
	cout << endl << "\t *TIP: Use 'more Testfile.txt' from the command line to preview the file" << endl;
}
int main(int argc, char* argv[])
{
	//syntax test.rxp test.txt allpoints x y z tgps
    try {

        if (argc >4) {
			string infilename=argv[1];
			string outfilename=argv[2];
			string pointsToPrint=argv[3];
			if (!(strcmp(argv[3],"allpoints")==0 || strcmp(argv[3],"allshots")==0 || strcmp(argv[3],"first")==0 || strcmp(argv[3],"last")==0))
			{
				cout << endl; showHelp();
				cout << endl << "Unknown argument 3 (type of points to return)" << endl << endl;
				return 0;
			}
			bool isGPS=true; 
			bool istdeclared=false;
			vector<char> params;
			string goodchars="abdRfhpesrtxyz012345nN";
			for (int i=4;i<argc;i++){
				string par=argv[i];
				
				if(strlen(argv[i])==1 && (goodchars.find(argv[i]) != std::string::npos))
				{
					params.push_back(argv[i][0]);
				}
				else if (strcmp(argv[i],"tgps")==0)
				{
					params.push_back('t');
					isGPS=true;
					if (istdeclared==true)//its already been used before
					{
						cout << endl; showHelp();
						cout << endl << "time can not be both internal time and GPS time... it's just how the Riegl Library is written" << endl << endl;
						return 1;
					}
					istdeclared=true;
				}
				else if (strcmp(argv[i],"tint")==0)
				{
					params.push_back('t');
					isGPS=false;
					if (istdeclared==true)//its already been used before
					{
						cout << endl; showHelp();
						cout << endl << "time can not be both internal time and GPS time... it's just how the Riegl Library is written" << endl << endl;
						return 1;
					}
					istdeclared=true;
				}
				else 
				{
					cout << endl; showHelp();
					cout << endl << "Unknown Parameter : " << argv[i] << endl << endl;
					return 1;
				}
			}
			//Print to screen what it is doing
			cout << "Converting: \t\t\t" << infilename << endl;
			cout << "OutputFile: \t\t\t" << outfilename << endl;
			cout << "Points Written to File: \t" << pointsToPrint << endl;
			cout << "Columns in Text File: \t\t";
			cout << argv[4] ;
			for (int i=5;i<argc;i++){
				cout << "," << argv[i];
			}
			cout << endl;

            // The basic_rconnection class contains the communication
            // protocol between the scanner or file and the program.
            shared_ptr<basic_rconnection> rc;

            // The static create function analyses the argument and
            // gives back a suitable class that is derived from
            // basic_rconnection and can handle the requested protocol.
            // The argument string is modelled using the common 'uri'
            // syntax, i.e. a protocol specifier such as 'file:' or
            // 'rdtp:" is followed by the 'resource location'.
            // E.g. 'file:C:/scans/test.rxp' would specify a file that
            // is stored on drive C in the scans subdirectory.
			string fileconn= string("file:")+string(argv[1]);
            rc = basic_rconnection::create(fileconn); //"rdtp://192.168.0.125/CURRENT"
            rc->open();

            // The decoder class scans off distinct packets from the
            // continuous data stream i.e. the rxp format and manages
            // the packets in a buffer.
            decoder_rxpmarker dec(rc);

            // The importer ( based on pointcloud and basic_packets class)
            // recognizes the packet types and calls into a distinct
            // function for each type. The functions are overidable
            // virtual functions, so a derived class can get a callback
            // per packet type.
			//write in binary format with no header
			ofstream outfile;
			bool isbinary=false;
			string filenameExt=outfilename.substr(outfilename.find_last_of(".") + 1);
			if (filenameExt == "bin"){
				outfile.open(outfilename,ios::out | ios::binary);
				isbinary=true;
				cout << "Writing in binary with no header" << endl;
			}
			else if (filenameExt == "txt")
			{

				outfile.open(outfilename);

				cout << "Writing to txt as ascii with a header" << endl;
				outfile << argv[4] ;
				for (int i=5;i<argc;i++){
					outfile << "," << argv[i];
				}
				outfile << endl;
			}
			else
			{
				outfile.open(outfilename);
				
				cout << "Unknown file extension... writing a comma delimited ascii (no header)" << endl;

			}

            importer     imp(outfile,isGPS,params,pointsToPrint,isbinary);
			
            // The buffer, despite its name is a structure that holds
            // pointers into the decoder buffer thereby avoiding
            // unnecessary copies of the data.
            buffer       buf;

            // This is the main loop, alternately fetching data from
            // the buffer and handing it over to the packets recognizer.
            // Please note, that there is no copy overhead for packets
            // which you do not need, since they will never be touched
            // if you do not access them.
            for ( dec.get(buf); !dec.eoi(); dec.get(buf) ) {
				try {
                imp.dispatch(buf.begin(), buf.end());
				}
				catch(exception& e) {
					cout << "ERROR: ";
					cerr << e.what() << endl;
				}
			}
			outfile.close();
            rc->close();
            return 0;
        }
		cout << endl;
		showHelp();
		cout << endl << "Not enough arguments passed" << endl << endl;
		return 1;

    }
    catch(exception& e) {
		cout << endl;
        cerr << e.what() << endl;
        return 1;
    }
    catch(...) {
        cerr << "unknown exception" << endl;
        return 1;
    }

    return 1;
}
