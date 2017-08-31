// Copyright 2016 Intelligent Robtics Group, NASA ARC

#include <cpu_monitor/cpu_monitor.h>

namespace cpu_monitor {

namespace {
constexpr char kProcStat[] = "/proc/stat";
constexpr char SysCpuPath[] = "/sys/devices/system/cpu";
constexpr char SysThermalPath[] = "/sys/class/thermal";
}  // namespace

CpuMonitor::CpuMonitor() :
  ff_util::FreeFlyerNodelet(),
  freq_cpus_(SysCpuPath, SysThermalPath),
  temperature_scale_(1.0),
  pub_queue_size_(10),
  update_freq_hz_(1),
  cpu_ave_load_limit_(95) {
}

CpuMonitor::~CpuMonitor() {
}

void CpuMonitor::Initialize(ros::NodeHandle *nh) {
  // First three letters of the node name specifies processor
  processor_name_ = GetName().substr(0, 3);

  config_params_.AddFile("management/cpu_monitor.config");
  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }

  reload_params_timer_ = nh->createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params_.CheckFilesUpdated(std::bind(&CpuMonitor::ReadParams, this));},
      false, true);

  // All state messages are latching
  cpu_state_pub_ = nh->advertise<ff_msgs::CpuStateStamped>(
                    TOPIC_MANAGEMENT_CPU_MONITOR_STATE, pub_queue_size_, true);

  // Timer for checking cpu stats. Timer is not one shot and start it right away
  stats_timer_ = nh->createTimer(ros::Duration(update_freq_hz_),
                                 &CpuMonitor::PublishStatsCallback,
                                 this, false, true);

  // Find the number of CPUs available to query
  ncpus_ = get_nprocs_conf();

  load_cpus_.resize(ncpus_ + 1);

  // Intialize cpu freq class
  if (!freq_cpus_.Init()) {
    ROS_FATAL("CPU Monitor: Cpu.init failed: %s", std::strerror(errno));
    exit(EXIT_FAILURE);
    return;
  }

  if (ncpus_ != freq_cpus_.GetNumCores()) {
    ROS_FATAL("CPU Monitor: Number of CPUs doesn't match!");
    exit(EXIT_FAILURE);
    return;
  }

  // Intialize cpu state message
  cpu_state_msg_.name = GetName();

  // Five load fields: nice, user, sys, virt, total
  cpu_state_msg_.load_fields.resize(5);
  cpu_state_msg_.ave_loads.resize(5);

  // Load locatioms won't change
  cpu_state_msg_.load_fields[0] = ff_msgs::CpuStateStamped::NICE;
  cpu_state_msg_.load_fields[1] = ff_msgs::CpuStateStamped::USER;
  cpu_state_msg_.load_fields[2] = ff_msgs::CpuStateStamped::SYS;
  cpu_state_msg_.load_fields[3] = ff_msgs::CpuStateStamped::VIRT;
  cpu_state_msg_.load_fields[4] = ff_msgs::CpuStateStamped::TOTAL;

  // Create as many cpu states as the number of cpus the freq class found
  cpu_state_msg_.cpus.resize(ncpus_);
  for (unsigned int i = 0; i < ncpus_; i++) {
    // Five load fields: nice, user, sys, virt, total
    cpu_state_msg_.cpus[i].loads.resize(5);
  }
}

bool CpuMonitor::ReadParams() {
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    ROS_FATAL("Error loading cpu monitor parameters.");
    return false;
  }

  // Get table for this processor
  config_reader::ConfigReader::Table processor_config(&config_params_,
                                                      processor_name_.c_str());

  // get udpate stats frequency
  if (!processor_config.GetInt("update_freq_hz", &update_freq_hz_)) {
    ROS_FATAL("CPU Monitor update frequency not specified!");
    return false;
  }

  if (!processor_config.GetPosReal("temperature_scale", &temperature_scale_)) {
    ROS_FATAL("CPU Monitor temperature scale not specified!");
    return false;
  }

  // get cpu ave load limit
  if (!config_params_.GetInt("cpu_ave_load_limit", &cpu_ave_load_limit_)) {
    ROS_FATAL("CPU Monitor cpu average load limit not specified!");
    return false;
  }

  // get cpu ave temp limit
  if (!config_params_.GetInt("cpu_temp_limit", &cpu_temp_limit_)) {
    ROS_FATAL("CPU Monitor cpu temp limit not specified!");
    return false;
  }

  return true;
}

int CpuMonitor::CollectLoadStats() {
  uint64_t user, nice, system, idle, iowait, irq, softirq,
           steal, guest, guest_nice,
           total, idle_all, system_all, virtual_all = 0;
  uint64_t total_period, user_period, nice_period, steal_period,
                guest_period, system_all_period;

  uint32_t cpuid = 0;
  char buffer[256];

  FILE * statf = fopen(kProcStat, "r");
  if (statf == NULL) {
    perror("fopen");
    return -1;
  }

  for (unsigned int i = 0; i < load_cpus_.size(); i++) {
    if (fgets(buffer, sizeof(buffer), statf) == NULL) {
      fprintf(stderr, "fgets screwed up\n");
      break;
    }

    if (i == 0) {
      sscanf(buffer, "cpu  %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64 \
                         " %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64 \
                         " %" SCNu64 " %" SCNu64 "", &user, &nice, &system,
                         &idle, &iowait, &irq, &softirq, &steal, &guest,
                          &guest_nice);
    } else {
      sscanf(buffer, "cpu%" SCNu32 " %" SCNu64 " %" SCNu64 " %" SCNu64 \
                       " %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64 \
                       " %" SCNu64 " %" SCNu64 " %" SCNu64 "", &cpuid, &user,
                       &nice, &system, &idle, &iowait, &irq, &softirq, &steal,
                       &guest, &guest_nice);
    }

    user -= guest;
    nice -= guest_nice;
    idle_all = idle + iowait;
    system_all = system + irq + softirq;
    virtual_all = guest + guest_nice;
    total = user + nice + system_all + idle_all + steal + virtual_all;

    Load *cpu = (&load_cpus_[i]);

    user_period = user - cpu->user_time;
    nice_period = nice - cpu->nice_time;
    // system_period = system - cpu->system_time;
    // idle_period = idle - cpu->idle_time;
    // io_period = iowait - cpu->io_time;
    // irq_period = irq - cpu->irq_time;
    // soft_irq_period = softirq - cpu->soft_irq_time;
    steal_period = steal - cpu->steal_time;
    guest_period = virtual_all - cpu->guest_time;
    system_all_period = system_all - cpu->system_all_time;
    // idle_all_period = idle_all - cpu->idle_all_time;
    total_period = total - cpu->total_time;

    cpu->user_time = user;
    cpu->nice_time = nice;
    cpu->system_time = system;
    cpu->idle_time = idle;
    cpu->io_time = iowait;
    cpu->irq_time = irq;
    cpu->soft_irq_time = softirq;
    cpu->steal_time = steal;
    cpu->guest_time = virtual_all;
    cpu->system_all_time = system_all;
    cpu->idle_all_time = idle_all;
    cpu->total_time = total;

    // This can happen if android turns the CPU off, for example.
    if (total_period == 0)
      total_period = 1;

    double totald = static_cast<double>(total_period);
    cpu->nice_percentage = nice_period / totald * 100.0d;
    cpu->user_percentage = user_period / totald * 100.0d;
    cpu->system_percentage = system_all_period / totald * 100.0d;
    cpu->virt_percentage = (guest_period + steal_period) / totald * 100.0d;
    cpu->total_percentage = cpu->nice_percentage +
                            cpu->user_percentage +
                            cpu->system_percentage +
                            cpu->virt_percentage;
  }

  fclose(statf);
  return 0;
}

void CpuMonitor::PublishStatsCallback(ros::TimerEvent const &te) {
  // Get cpu load stats first
  if (CollectLoadStats() < 0) {
    ROS_FATAL("CPU node unable to get load stats!");
    return;
  }

  // Index 0 in the load cpus array refers to the average of all the cpus, cpu0
  // starts at index 1
  // Add average cpu stats to the state message
  // see init function for order
  cpu_state_msg_.ave_loads[0] = load_cpus_[0].nice_percentage;
  cpu_state_msg_.ave_loads[1] = load_cpus_[0].user_percentage;
  cpu_state_msg_.ave_loads[2] = load_cpus_[0].system_percentage;
  cpu_state_msg_.ave_loads[3] = load_cpus_[0].virt_percentage;
  cpu_state_msg_.ave_loads[4] = load_cpus_[0].total_percentage;

  // Add the individual cpu loads to cpu state msg
  for (unsigned int i = 0; i < cpu_state_msg_.cpus.size(); i++) {
    // cpu0 starts at index 1 in cpu loads array
    cpu_state_msg_.cpus[i].loads[0] = load_cpus_[(i + 1)].nice_percentage;
    cpu_state_msg_.cpus[i].loads[1] = load_cpus_[(i + 1)].user_percentage;
    cpu_state_msg_.cpus[i].loads[2] = load_cpus_[(i + 1)].system_percentage;
    cpu_state_msg_.cpus[i].loads[3] = load_cpus_[(i + 1)].virt_percentage;
    cpu_state_msg_.cpus[i].loads[4] = load_cpus_[(i + 1)].total_percentage;
  }

  // Check to see if the total percentage is greater than the fault threshold
  if (load_cpus_[0].total_percentage > cpu_ave_load_limit_) {
    std::string msg = "CPU ave load is " +
                      std::to_string(load_cpus_[0].total_percentage) +
                      " which is greater than " +
                      std::to_string(cpu_ave_load_limit_) + ".";
    this->AssertFault("LOAD_TOO_HIGH", msg);
  } else {
    this->ClearFault("LOAD_TOO_HIGH");
  }

  // Get cpu temperature stats
  cpu_state_msg_.temp = freq_cpus_.GetTemperature(temperature_scale_);

  // Check to see if the temperature is greater than the fault threshold
  if (cpu_state_msg_.temp > cpu_temp_limit_) {
    std::string msg = "CPU average temperature is " +
                      std::to_string(cpu_state_msg_.temp) +
                      " which is greater than " +
                      std::to_string(cpu_temp_limit_) + ".";
    this->AssertFault("TEMPERATURE_TOO_HIGH", msg);
  } else {
    this->ClearFault("TEMPERATURE_TOO_HIGH");
  }

  // Get cpu frequency stats
  for (unsigned int i = 0; i < freq_cpus_.GetNumCores(); i++) {
    Core *c = (freq_cpus_.GetCores())[i];

    cpu_state_msg_.cpus[i].enabled = c->IsOn();
    cpu_state_msg_.cpus[i].max_frequency = c->GetMaxFreq();
    cpu_state_msg_.cpus[i].frequency = c->IsOn() ? c->GetCurFreq() : 0;
  }

  // Send cpu stats
  cpu_state_msg_.header.stamp = ros::Time::now();
  cpu_state_pub_.publish(cpu_state_msg_);
}

}  // namespace cpu_monitor

PLUGINLIB_EXPORT_CLASS(cpu_monitor::CpuMonitor, nodelet::Nodelet)
