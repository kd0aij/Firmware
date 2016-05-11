/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "logger.h"

#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/uORBTopics.h>
#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <uORB/topics/bw_test.h>

//#define DBGPRINT //write status output every few seconds

#if defined(DBGPRINT)
// needed for mallinfo
#if defined(__PX4_POSIX) && !defined(__PX4_DARWIN)
#include <malloc.h>
#endif /* __PX4_POSIX */

// struct mallinfo not available on OSX?
#if defined(__PX4_DARWIN)
#undef DBGPRINT
#endif /* defined(__PX4_DARWIN) */
#endif /* defined(DBGPRINT) */

using namespace px4::logger;

int logger_main(int argc, char *argv[])
{
	// logger currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Logger only works on little endian!\n");
		return 1;
	}

	if (argc < 2) {
		PX4_INFO("usage: logger {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (logger_ptr != nullptr) {
			PX4_INFO("already running");
			return 1;
		}

		if (OK != Logger::start((char *const *)argv)) {
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (logger_ptr == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		delete logger_ptr;
		logger_ptr = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (logger_ptr) {
			logger_ptr->status();
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	Logger::usage("unrecognized command");
	return 1;
}

namespace px4
{
namespace logger
{

void Logger::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_INFO("usage: logger {start|stop|status} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
		 "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
		 "\t-b\tLog buffer size in KiB, default is 12\n"
		 "\t-e\tEnable logging by default (if not, can be started by command)\n"
		 "\t-a\tLog only when armed (can be still overriden by command)\n"
		 "\t-t\tUse date/time for naming log directories and files\n"
		 "\t-x\tExtended logging");
}

int Logger::start(char *const *argv)
{
	ASSERT(logger_task == -1);

	/* start the task */
	logger_task = px4_task_spawn_cmd("logger",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 3100,
					 (px4_main_t)&Logger::run_trampoline,
					 (char *const *)argv);

	if (logger_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

void Logger::status()
{
	if (!_enabled) {
		PX4_INFO("Running, but not logging");

	} else {
		PX4_INFO("Running");

		float kibibytes = (float)_writer.get_total_written() / 1024.0f;
		float mebibytes = kibibytes / 1024.0f;
		float seconds = ((float)(hrt_absolute_time() - _start_time)) / 1000000.0f;

		PX4_INFO("Wrote %4.2f MiB (avg %5.2f KiB/s)", (double)mebibytes, (double)(kibibytes / seconds));
		PX4_INFO("Dropouts: %zu (max len: %.3f s), max used buffer: %zu / %zu B",
			 _write_dropouts, (double)_max_dropout_duration, _high_water, _writer.get_buffer_size());
//		_high_water = 0;
//		_max_dropout_duration = 0.f;
	}
}

void Logger::run_trampoline(int argc, char *argv[])
{
	unsigned log_interval = 3500;
	int log_buffer_size = 12 * 1024;
	bool log_on_start = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "r:b:eatx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, NULL, 10);

				if (r <= 0) {
					r = 1;
				}

				log_interval = 1e6 / r;
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, NULL, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		logger_task = -1;
		return;
	}

	logger_ptr = new Logger(log_buffer_size, log_interval, log_on_start);

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
	struct mallinfo alloc_info = mallinfo();
	warnx("largest free chunk: %d bytes", alloc_info.mxordblk);
	warnx("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

	if (logger_ptr == nullptr) {
		PX4_WARN("alloc failed");

	} else {
		logger_ptr->run();
	}
}

enum class MessageType : uint8_t {
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	PARAMETER = 'P',
};

/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)
struct message_format_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::FORMAT);
	uint16_t msg_size;

	uint8_t msg_id;
	uint16_t format_len;
	char format[2096];
};

struct message_data_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::DATA);
	uint16_t msg_size;

	uint8_t msg_id;
	uint8_t multi_id;
};

// currently unused
struct message_info_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::INFO);
	uint16_t msg_size;

	uint8_t key_len;
	char key[255];
};

struct message_parameter_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	uint16_t msg_size;

	uint8_t key_len;
	char key[255];
};
#pragma pack(pop)


static constexpr size_t MAX_DATA_SIZE = 1024;

Logger::Logger(size_t buffer_size, unsigned log_interval, bool log_on_start) :
	_log_on_start(log_on_start),
	_writer(buffer_size),
	_log_interval(log_interval)
{
}

Logger::~Logger()
{
	if (logger_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned int i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 200) {
				px4_task_delete(logger_task);
				logger_task = -1;
				break;
			}
		} while (logger_task != -1);
	}
}

int Logger::add_topic(const orb_metadata *topic)
{
	int fd = -1;

	if (topic->o_size > MAX_DATA_SIZE) {
		PX4_WARN("skip topic %s, data size is too large: %zu (max is %zu)", topic->o_name, topic->o_size, MAX_DATA_SIZE);
		return -1;

	}

	size_t fields_len = strlen(topic->o_fields);

	if (fields_len > sizeof(message_format_s::format)) {
		PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
			 sizeof(message_format_s::format));

		return -1;
	}

	fd = orb_subscribe(topic);

	if (fd < 0) {
		PX4_WARN("logger: orb_subscribe failed");
		return -1;
	}

	if (!_subscriptions.push_back(LoggerSubscription(fd, topic))) {
		PX4_WARN("logger: failed to add topic. Too many subscriptions");
		orb_unsubscribe(fd);
		fd = -1;
	}

	return fd;
}

int Logger::add_topic(const char *name, unsigned interval = 0)
{
	const orb_metadata **topics = orb_get_topics();
	int fd = -1;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			fd = add_topic(topics[i]);
			PX4_INFO("logging topic: %zu, %s", i, topics[i]->o_name);
			break;
		}
	}

	if (fd >= 0 && interval != 0) {
		orb_set_interval(fd, interval);
	}

	return fd;
}

bool Logger::copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer,
				   uint64_t *time_last_checked)
{
	bool updated = false;

	// only try to subscribe to topic if this is the first time
	// after that just check after a certain interval to avoid high cpu usage
	if (*handle < 0 && (*time_last_checked == 0 || hrt_elapsed_time(time_last_checked) > TRY_SUBSCRIBE_INTERVAL)) {
		//if (multi_instance == 1) warnx("checking instance 1 of topic %s", topic->o_name);
		*time_last_checked = hrt_absolute_time();

		if (OK == orb_exists(topic, multi_instance)) {
			*handle = orb_subscribe_multi(topic, multi_instance);

			//warnx("subscribed to instance %d of topic %s", multi_instance, topic->o_name);

			/* copy first data */
			if (*handle >= 0) {
				orb_copy(topic, *handle, buffer);
				updated = true;
			}
		}

	} else if (*handle >= 0) {
		orb_check(*handle, &updated);

		if (updated) {
			orb_copy(topic, *handle, buffer);
		}
	}

	return updated;
}

void Logger::run()
{
#ifdef DBGPRINT
	struct mallinfo alloc_info = {};
#endif /* DBGPRINT */

	PX4_INFO("logger started");

	struct bw_test_s* bw_testp = (struct bw_test_s*)malloc(sizeof(struct bw_test_s));
	bw_testp->timestamp = hrt_absolute_time();
	const char * test_buf_id = "bw_test_record";
	memcpy(&bw_testp->data, test_buf_id, strlen(test_buf_id));
	for (int i=strlen(test_buf_id); i<(sizeof(bw_testp->data)/sizeof(int8_t)); i++) {
		bw_testp->data[i] = i & 0xFF;
	}
//	orb_advert_t bw_test_pub = orb_advertise(ORB_ID(bw_test), bw_testp);
//	int pstat = orb_publish(ORB_ID(bw_test), bw_test_pub, bw_testp);
//	PX4_INFO("pub stat: %d", pstat);
//
//	add_topic("bw_test");

	int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret == 0) {
		PX4_INFO("log root dir created: %s", LOG_ROOT);

	} else if (errno != EEXIST) {
		PX4_ERR("failed creating log root dir: %s", LOG_ROOT);
		return;
	}

	if (!(_vehicle_status_sub = new uORB::Subscription<vehicle_status_s>(ORB_ID(vehicle_status)))) {
		PX4_ERR("Failed to allocate subscription");
		return;
	}

	if (!(_parameter_update_sub = new uORB::Subscription<parameter_update_s>(ORB_ID(parameter_update)))) {
		delete _vehicle_status_sub;
		_vehicle_status_sub = nullptr;
		PX4_ERR("Failed to allocate subscription");
		return;
	}


//	add_topic("sensor_gyro", 0);
//	add_topic("sensor_accel", 0);
//	add_topic("vehicle_rates_setpoint", 10);
//	add_topic("vehicle_attitude_setpoint", 10);
//	add_topic("vehicle_attitude", 0);
//	add_topic("actuator_outputs", 50);
//	add_topic("battery_status", 100);
//	add_topic("vehicle_command", 100);
//	add_topic("actuator_controls", 10);
//	add_topic("vehicle_local_position_setpoint", 200);
//	add_topic("rc_channels", 20);
////	add_topic("ekf2_innovations", 20);
//	add_topic("commander_state", 100);
//	add_topic("vehicle_local_position", 200);
//	add_topic("vehicle_global_position", 200);
//	add_topic("system_power", 100);
//	add_topic("servorail_status", 200);
//	add_topic("mc_att_ctrl_status", 50);
////	add_topic("control_state");
////	add_topic("estimator_status");
//	add_topic("vehicle_status", 200);

	if (!_writer.init()) {
		PX4_ERR("logger: init of writer failed");
		return;
	}

	int ret = _writer.thread_start(_writer_thread);

	if (ret) {
		PX4_ERR("logger: failed to create writer thread (%i)", ret);
		return;
	}

	_task_should_exit = false;

#ifdef DBGPRINT
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;
#endif /* DBGPRINT */

	// we start logging immediately
	// the case where we wait with logging until vehicle is armed is handled below
	if (_log_on_start) {
		start_log();
	}

	uint32_t total_bytes = 0;
	hrt_abstime timer_start = hrt_absolute_time();

	/* every log_interval usec, check for orb updates */
	while (!_task_should_exit) {
//		// publish data
//		bw_testp->timestamp = hrt_absolute_time();
//		pstat = orb_publish(ORB_ID(bw_test), bw_test_pub, bw_testp);

		if (_enabled) {

			bool data_written = false;

			/* wait for lock on log buffer */
			_writer.lock();

			// fill ringbuffer at desired rate
			hrt_abstime delWrite = hrt_elapsed_time(&_lastWrite);
			uint16_t writeLen = 150 * 1000 * (float)delWrite / 1.0e6f;
			if (writeLen >= sizeof(bw_testp->data)) {
				warnx("writeLen %d > buffer size: %zu", writeLen, sizeof(bw_testp->data));
				writeLen = sizeof(bw_testp->data);
			}


			bw_testp->timestamp = hrt_absolute_time();
			if (_writer.write(bw_testp, writeLen)) {

				total_bytes += writeLen;
				_lastWrite = hrt_absolute_time();

				if (_dropout_start) {
					float dropout_duration = (float)(hrt_elapsed_time(&_dropout_start) / 1000) / 1.e3f;

					if (dropout_duration > _max_dropout_duration) {
						_max_dropout_duration = dropout_duration;
					}

					_dropout_start = 0;
				}

				data_written = true;

			} else {

				if (!_dropout_start) {
					_dropout_start = hrt_absolute_time();
					++_write_dropouts;
					_high_water = 0;
				}

				break;	// Write buffer overflow, skip this record
			}

			if (!_dropout_start && _writer.get_buffer_fill_count() > _high_water) {
				_high_water = _writer.get_buffer_fill_count();
			}

			/* release the log buffer */
			_writer.unlock();

			/* notify the writer thread if data is available */
			if (data_written) {
				_writer.notify();
			}

			double deltat = (double)(hrt_elapsed_time(&timer_start))  * 1e-6;

			if (deltat > 10.0) {
				double throughput = total_bytes / deltat;
				warnx("%8.1f kB/s, %zu highWater", throughput / 1.e3, _high_water);

				total_bytes = 0;
				timer_start = hrt_absolute_time();
			}

		}

//		hrt_abstime collectDuration = hrt_elapsed_time(&_nextCollect);
//		hrt_abstime now = hrt_absolute_time();
//		// "schedule" next collection N intervals from _starttime
//		_nextCollect += _log_interval;
//		if (collectDuration < _log_interval) {
//			hrt_abstime remInterval = _log_interval - collectDuration;
////			warnx("now: %llu, duration: %llu, nextCollect: %llu, remInterval: %llu",
////					now, collectDuration, _nextCollect, remInterval);
//			usleep(remInterval - 100);
//		} else {
//			_nextCollect = now + _log_interval;
//			warnx("timing slip: now: %llu, duration: %llu, nextCollect: %llu, _log_interval: %d",
//					now, collectDuration, _nextCollect, _log_interval);
//			usleep(_log_interval - 100);
//		}
		usleep(_log_interval);
	}

	delete _vehicle_status_sub;
	delete _parameter_update_sub;

	// stop the writer thread
	_writer.thread_stop();

	_writer.notify();

	// wait for thread to complete
	ret = pthread_join(_writer_thread, NULL);

	if (ret) {
		PX4_WARN("join failed: %d", ret);
	}

	logger_task = -1;
}

int Logger::create_log_dir()
{
	/* create dir on sdcard if needed */
	uint16_t dir_number = 1; // start with dir sess001
	int mkdir_ret;

	/* look for the next dir that does not exist */
	while (dir_number <= MAX_NO_LOGFOLDER) {
		/* format log dir: e.g. /fs/microsd/sess001 */
		sprintf(_log_dir, "%s/sess%03u", LOG_ROOT, dir_number);
		mkdir_ret = mkdir(_log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == 0) {
			PX4_INFO("log dir created: %s", _log_dir);
			break;

		} else if (errno != EEXIST) {
			PX4_WARN("failed creating new dir: %s", _log_dir);
			return -1;
		}

		/* dir exists already */
		dir_number++;
		continue;
	}

	if (dir_number >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		PX4_WARN("all %d possible dirs exist already", MAX_NO_LOGFOLDER);
		return -1;
	}

	/* print logging path, important to find log file later */
	//mavlink_and_console_log_info(mavlink_fd, "[sdlog2] log dir: %s", log_dir);
	return 0;
}

bool Logger::file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int Logger::get_log_file_name(char *file_name, size_t file_name_size)
{
	uint16_t file_number = 1; // start with file log001

	/* look for the next file that does not exist */
	while (file_number <= MAX_NO_LOGFILE) {
		/* format log file path: e.g. /fs/microsd/sess001/log001.ulg */
		snprintf(file_name, file_name_size, "%s/log%03u.ulg", _log_dir, file_number);

		if (!file_exist(file_name)) {
			break;
		}

		file_number++;
	}

	if (file_number > MAX_NO_LOGFILE) {
		/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
		//mavlink_and_console_log_critical(mavlink_fd, "[sdlog2] ERR: max files %d", MAX_NO_LOGFILE);
		return -1;
	}

	return 0;
}

void Logger::start_log()
{
	PX4_WARN("start log");

	if (create_log_dir()) {
		return;
	}

	char file_name[64] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		return;
	}

	_writer.start_log(file_name);
//	write_version();
//	write_formats();
//	write_parameters();
	_writer.notify();
	_enabled = true;
	_start_time = hrt_absolute_time();
	_nextCollect = _start_time;
	_lastWrite = _start_time;
}

void Logger::stop_log()
{
	_enabled = false;
	_writer.stop_log();
}

bool Logger::write_wait(void *ptr, size_t size)
{
	while (!_writer.write(ptr, size)) {
		_writer.unlock();
		_writer.notify();
		usleep(_log_interval);
		_writer.lock();
	}

	return true;
}

void Logger::write_formats()
{
	_writer.lock();
	message_format_s msg;

	int msg_id = 0;

	for (LoggerSubscription &sub : _subscriptions) {
		msg.msg_id = msg_id;
		msg.format_len = snprintf(msg.format, sizeof(msg.format), "%s", sub.metadata->o_fields);
		size_t msg_size = sizeof(msg) - sizeof(msg.format) + msg.format_len;
		msg.msg_size = msg_size - 3;

		write_wait(&msg, msg_size);

		msg_id++;
	}

	_writer.unlock();
}

/* write info message */
void Logger::write_info(const char *name, const char *value)
{
	_writer.lock();
	uint8_t buffer[sizeof(message_info_header_s)];
	message_info_header_s *msg = reinterpret_cast<message_info_header_s *>(buffer);
	msg->msg_type = static_cast<uint8_t>(MessageType::INFO);

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg->key_len = snprintf(msg->key, sizeof(msg->key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(*msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg->msg_size = msg_size - 3;

		write_wait(buffer, msg_size);
	}

	_writer.unlock();
}

/* write version info messages */
void Logger::write_version()
{
	write_info("ver_sw", PX4_GIT_VERSION_STR);
	write_info("ver_hw", HW_ARCH);
}

void Logger::write_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(message_parameter_header_s) + sizeof(param_value_u)];
	message_parameter_header_s *msg = reinterpret_cast<message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// get next parameter which is invalid OR used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// save parameters which are valid AND used
		if (param != PARAM_INVALID) {
			/* get parameter type and size */
			const char *type_str;
			param_type_t type = param_type(param);
			size_t value_size = 0;

			switch (type) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			/* format parameter key (type and name) */
			msg->key_len = snprintf(msg->key, sizeof(msg->key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

			/* copy parameter value directly to buffer */
			param_get(param, &buffer[msg_size]);
			msg_size += value_size;

			msg->msg_size = msg_size - 3;

			write_wait(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(message_parameter_header_s) + sizeof(param_value_u)];
	message_parameter_header_s *msg = reinterpret_cast<message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// get next parameter which is invalid OR used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// log parameters which are valid AND used AND unsaved
		if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
			warnx("logging change to parameter %s", param_name(param));

			/* get parameter type and size */
			const char *type_str;
			param_type_t type = param_type(param);
			size_t value_size = 0;

			switch (type) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			/* format parameter key (type and name) */
			msg->key_len = snprintf(msg->key, sizeof(msg->key), "%s %s ", type_str, param_name(param));
			size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

			/* copy parameter value directly to buffer */
			param_get(param, &buffer[msg_size]);
			msg_size += value_size;

			/* msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size */
			msg->msg_size = msg_size - 3;

			write_wait(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

}
}
