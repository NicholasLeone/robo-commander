#include <boost/program_options.hpp>


using namespace std;
namespace po = boost::program_options;

void test_options(){
	try{
	          po::options_description desc("Options");
	          desc.add_options()
	               ("help", "show help message")
	               ("keyboard", "Keyboard Control of bot")
	               ("paraminfo", "INFO: Displays hard-coded variables of interest")
	               ("pulse", po::value<int>()->implicit_value(12600),
	                    "MOTOR: Tests control of single motor at specified pulsewidth [us]")
	               ("speed", po::value<double>()->implicit_value(1.0),
	                    "MOTOR: Controls single motor at a specified speed ratio [ -1 < ratio < 1 ]")
	               ("max", "MOTOR: Sets PWM pulse at maximum pulse [us]")
	               ("min", "MOTOR: Sets PWM pulse at minimum pulse [us]")
	               ("wheels", po::value<int>()->implicit_value(12600),
	                    "MOTOR: Tests control of two motors simultaneously at specified pulsewidth [us]")
	               ("fwdback", po::value<double>()->implicit_value(1.0),
	                    "MOTOR: Controls two motors at a specified speed ratio in both + and - direction [ -1 < ratio < 1 ]")
	               ("pid", po::value<double>()->implicit_value(1.0),
	                    "PID: Tests PID performance for reaching a target speed ratio [ -1 < ratio < 1 ]")
	               ("servo", po::value<int>(), "SERVO: Tests Servo functions")
	               // ("servos", po::value<std::vector<int> >()->multitoken(), "SERVO: Tests Multiple Servos functions")
	          ;
	
	          po::variables_map vm;
	          po::store(po::parse_command_line(argc, argv, desc,
	               po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
	          po::notify(vm);
	
	          vector<int> opts;
	          if (vm.count("help") || argc==1){
	               std::cout << desc << "\n";
	               return 1;
	          }
	
	          if (vm.count("keyboard") || vm.count("k"))
	               keyboardControl();
	          if (vm.count("pulse") || vm.count("p"))
	               testPulse(vm["pulse"].as<int>());
	          if (vm.count("speed") || vm.count("s"))
	               testSpeed(vm["speed"].as<double>());
	          if (vm.count("max") || vm.count("M"))
	               testMaxPulse();
	          if (vm.count("min") || vm.count("m"))
	               testMinPulse();
	          if (vm.count("wheels") || vm.count("w"))
	               testWheels(vm["wheels"].as<int>());
	          if (vm.count("fwdback") || vm.count("f"))
	               testFwdBack(vm["fwdback"].as<double>());
	          if (vm.count("pid"))
	               testPID(vm["pid"].as<double>());
	          if (vm.count("servo"))
	               testServoPulse(vm["servo"].as<int>());
	          // if (!vm["servo"].empty() && (opts = vm["servo"].as<vector<double> >()).size() == 3)
	          //      testServos(vm["servo"].as<vector<double> >());
	
	     } catch(std::exception& e) {std::cerr << "error: " << e.what() << "\n"; return 1;
	     } catch(...) { std::cerr << "Exception of unknown type!\n";}
}