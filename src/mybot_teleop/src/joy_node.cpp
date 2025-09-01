
#define JS_READ_HZ 150

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>

#include <atomic>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std;
using namespace std::chrono;

class Joy : public rclcpp::Node {
public:
	Joy() 
		: 
		Node("joy_node"),
		reading_state(false),
		js_fd(-1)
	{
		joy_msg.header.frame_id = "joy"; //TODO What is anyway.

		int device_id;
		this->get_parameter_or<int>(
			"device_id",
			device_id,
			0
		);

		this->get_parameter_or<float>(
			"deadzone",
			deadzone,
			0.1
		);

		float autorepeat_rate;
		this->get_parameter_or<float>(
			"autorepeat_rate",
			autorepeat_rate,
			20.0
		);


		ostringstream oss;
		oss << "/dev/input/js" << device_id;
		js_fn = oss.str();
				
		joy__pub = this->create_publisher<sensor_msgs::msg::Joy>(
			"joy",
			rclcpp::QoS(rclcpp::KeepLast(10))
		);


		js_fd__thread = thread(
			std::bind(&Joy::js_fd__task, this)
		);
		poll_msg__timer = this->create_wall_timer(
			std::chrono::milliseconds(int(1000/JS_READ_HZ)),
			std::bind(&Joy::poll_msg__cb, this)
		);
		if(autorepeat_rate > 0){
			autorepeat__timer = this->create_wall_timer(
				std::chrono::milliseconds(int(1000/autorepeat_rate)),
				std::bind(&Joy::autorepeat__cb, this)
			);
		}

	}

	~Joy(){

		// Close the device file
		close(js_fd);

		//TODO Kill thread.
		//js_fd__thread.join();
	}

protected:
	atomic_bool reading_state;

	float deadzone;
	string js_fn;
	int js_fd;

	std::thread js_fd__thread;
	rclcpp::TimerBase::SharedPtr poll_msg__timer;
	rclcpp::TimerBase::SharedPtr autorepeat__timer;
	
	std::mutex msg_mutex; // for new_msg & joy_msg
	bool new_msg;
	sensor_msgs::msg::Joy joy_msg;

	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy__pub;

	std::chrono::time_point<std::chrono::steady_clock> t_prev;


	void js_fd__task() {
		while(true){
			if(!reading_state.load(std::memory_order_acquire)){
				// Opening state.
				//TODO Try reconnect on every sec. Just some delay of 1 sec.
				js_fd = open(js_fn.c_str(), O_RDONLY);
				if (js_fd == -1) {
					RCLCPP_ERROR(
						this->get_logger(),
						"Error opening joystick dev \"%s\": errno %d %s",
						js_fn.c_str(),
						errno,
						strerror(errno)
					);
					this_thread::sleep_for(seconds(1));
				}else{
					RCLCPP_INFO(this->get_logger(), "Successful opening \"%s\"", js_fn.c_str());

					int N_axes, N_buttons;
					ioctl(js_fd, JSIOCGAXES, &N_axes);
					ioctl(js_fd, JSIOCGBUTTONS, &N_buttons);

					lock_guard<mutex> lock(msg_mutex);
					// Resize and set to 0
					joy_msg.axes.resize(N_axes, 0.0);
					joy_msg.buttons.resize(N_buttons, 0);

					reading_state.store(true, std::memory_order_release);
				}
			}else{
				// Reading state.
				struct js_event js_event_data;

				// Continuously read joystick events
				if(read(js_fd, &js_event_data, sizeof(struct js_event)) == sizeof(struct js_event)){

					lock_guard<mutex> lock(msg_mutex);

					// Process the event based on its type
					if(js_event_data.type & JS_EVENT_BUTTON){
						joy_msg.buttons[js_event_data.number] = js_event_data.value;

						RCLCPP_DEBUG(
							this->get_logger(),
							"Button %d %s (value: %d)",
							js_event_data.number,
							(js_event_data.value == 0) ? "released" : "pressed",
							js_event_data.value
						);
					}else if(js_event_data.type & JS_EVENT_AXIS){
						// Normalize and invert.
						float axis = -float(js_event_data.value) / 32767;

						// Zero it if off the center.
						if(abs(axis) < deadzone){
							axis = 0;
						}

						joy_msg.axes[js_event_data.number] = axis;

						RCLCPP_DEBUG(
							this->get_logger(),
							"Axis %d moved (value: %d)",
							js_event_data.number,
							js_event_data.value
						);
					}else if(js_event_data.type & JS_EVENT_INIT){
						RCLCPP_DEBUG(
							this->get_logger(),
							"Initial state event (type: %d, number: %d, value: %d)",
							js_event_data.type, js_event_data.number, js_event_data.value
						);

						//TODO What a heac is this? It is never called.
					}

					new_msg = true;
				}else{
					RCLCPP_WARN(
						this->get_logger(),
						"Cannot read \"%s\": errno %d %s",
						js_fn.c_str(),
						errno,
						strerror(errno)
					);
					//TODO Test if joypad is plug out, and reading_state.store(false, std::memory_order_release);
				}

				this_thread::sleep_for(milliseconds(int(1/JS_READ_HZ)));
			}
		}
	}

	void poll_msg__cb() {
		if(reading_state.load(std::memory_order_acquire)){
			// Reading state.
			lock_guard<mutex> lock(msg_mutex);
			new_msg = false; // Msg read.
			
			joy_msg.header.stamp = this->get_clock()->now();
			//TODO Why not publish from thread?
			joy__pub->publish(joy_msg);
			//TODO Do not publish from here if have autorepeat?
		}
	}

	void autorepeat__cb() {
		if(reading_state.load(std::memory_order_acquire)){
			// Reading state.
			lock_guard<mutex> lock(msg_mutex);

			joy_msg.header.stamp = this->get_clock()->now();
			joy__pub->publish(joy_msg);
		}
	}


};


int main(int argc, char * argv[]) {

	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;


	auto joy_node = std::make_shared<Joy>();

	executor.add_node(joy_node);
	executor.spin();

	rclcpp::shutdown();
}