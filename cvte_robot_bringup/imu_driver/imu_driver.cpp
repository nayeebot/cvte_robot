#include <asm/types.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/netlink.h>
#include <linux/socket.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <thread>

#define IOCTRL_IMU42688_INIT _IOW('p', 0x01, uint8_t)
#define IOCTRL_GET_IMU42688_DATA _IOWR('p', 0x02, uint8_t)
#define NETLINK_TEST (25)
#define MAX_PAYLOAD (1024)
#define TEST_PID (100)

typedef short int i16_t;

struct icm42688 {
  i16_t acc_x;
  i16_t acc_y;
  i16_t acc_z;
  i16_t gyro_x;
  i16_t gyro_y;
  i16_t gyro_z;
  struct timeval c_time_value;
};

class IcmKoReader {
 public:
  IcmKoReader() { init(); }

  ~IcmKoReader() {
    start_thread_ = false;
    std::cout << "close icm ko reader class" << std::endl;

    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
    std::cout << "close icm ko reader class success" << std::endl;
  }

  void init() { imu_thread_ = std::thread(&IcmKoReader::work, this); }

  bool work() {
    for (int i = 0; i < 10; ++i) {
      fd_ = open("/dev/tdk_icm42688", O_RDWR);
      std::cout << "try open /dev/tdk_icm42688 at time:" << i << std::endl;
      if (fd_ > 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "open /dev/tdk_icm42688 fd: " << fd_ << std::endl;

    if (fd_ < 0) {
      std::cout << "open /dev/tdk_icm42688 error,please check hardware"
                << std::endl;
      return false;
    }

    int ret = ioctl(fd_, IOCTRL_IMU42688_INIT, (long)len_);

    if (ret == -1) {
      std::cout << "ioctl /dev/tdk_icm42688 failed,please check hard ware "
                << ret << std::endl;
      return false;
    }
    sleep(3);  // init time

    int sock_fd = -1;
    sock_fd = netlink_create_socket();
    if (sock_fd == -1) {
      std::cout << "socket creat error! sock_fd:" << sock_fd << std::endl;
      return false;
    }
    std::cout << "netlink_create_socket sock_fd: " << sock_fd << std::endl;

    if (netlink_bind(sock_fd) < 0) {
      close(sock_fd);
      std::cout << "netlink_bind error!" << std::endl;
      return false;
    }
    fd_set readfds, testfds;
    FD_ZERO(&readfds);
    FD_SET(sock_fd, &readfds);  // 将服务器端socket加入到集合中

    netlink_send_message(sock_fd, (unsigned char *)"h", 2, 0, 0);

    unsigned char buf[MAX_PAYLOAD];
    float gyro_sensitivity_ = 16.4;
    float acc_sensitivity_ = 4096.0;
    double icm42688_kGravity = 9.81;
    const double degree2rad_value = 1 * M_PI / 180.0;

    double gyro_k = degree2rad_value / gyro_sensitivity_;
    double acc_k = icm42688_kGravity / acc_sensitivity_;

    std::cout << "tdk_icm42688 read thread start" << std::endl;
    struct timeval wait_time_select;

    double gyro_x = 0, gyro_y = 0, gyro_z = 0;
    double acc_x = 0, acc_y = 0, acc_z = 0;
    int nanosec = 0, sec = 0;

    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    while (start_thread_) {
      testfds = readfds;
      wait_time_select.tv_sec = 0;
      wait_time_select.tv_usec = 10000;
      int select_ret = select(FD_SETSIZE, &testfds, (fd_set *)0, (fd_set *)0,
                              &wait_time_select);
      if (select_ret != 0) {
        netlink_recv_message(sock_fd, buf, &len_);
      }

      len_ = ioctl(fd_, IOCTRL_GET_IMU42688_DATA, (long)&arr_icm_);
      num_ = len_ / sizeof(struct icm42688);
      for (int index = 0; index < num_; index++) {
        gyro_x = arr_icm_[index].gyro_x * gyro_k;
        gyro_y = arr_icm_[index].gyro_y * gyro_k;
        gyro_z = arr_icm_[index].gyro_z * gyro_k;

        acc_x = acc_k * arr_icm_[index].acc_x;
        acc_y = acc_k * arr_icm_[index].acc_y;
        acc_z = acc_k * arr_icm_[index].acc_z;

        nanosec = arr_icm_[index].c_time_value.tv_usec * 1000.0;
        sec = arr_icm_[index].c_time_value.tv_sec;

        sensor_msgs::Imu imu_msg;

        imu_msg.header.stamp.sec = sec;
        imu_msg.header.stamp.nsec = nanosec;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;

        imu_msg.linear_acceleration.x = acc_x;
        imu_msg.linear_acceleration.y = acc_y;
        imu_msg.linear_acceleration.z = acc_z;
        imu_pub.publish(imu_msg);
      }
    }

    close(sock_fd);
    std::cout << "IcmKoReader close sock_fd" << std::endl;

    close(fd_);
    std::cout << "IcmKoReader read data thread exit and close fd_ "
              << std::endl;

    return true;
  }

 private:
  bool start_thread_ = true;
  int fd_ = -1;
  int len_ = 0;
  int num_ = 0;
  std::thread imu_thread_;
  struct icm42688 arr_icm_[301] = {0};

  int netlink_create_socket(void) {
    return socket(AF_NETLINK, SOCK_RAW, NETLINK_TEST);
  }

  int netlink_bind(int sock_fd) {
    struct sockaddr_nl addr;
    memset(&addr, 0, sizeof(struct sockaddr_nl));
    addr.nl_family = AF_NETLINK;
    addr.nl_pid = TEST_PID;
    addr.nl_groups = 0;
    return bind(sock_fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_nl));
  }

  int netlink_send_message(int sock_fd, const unsigned char *message, int len,
                           unsigned int pid, unsigned int group) {
    struct nlmsghdr *nlh = NULL;
    char nlh_payload[NLMSG_SPACE(MAX_PAYLOAD)];
    nlh = (struct nlmsghdr *)&nlh_payload[0];
    struct sockaddr_nl dest_addr;

    if (!message) {
      return -1;
    }

    nlh->nlmsg_len = NLMSG_SPACE(len);
    nlh->nlmsg_pid = TEST_PID;
    nlh->nlmsg_flags = 0;
    memcpy(NLMSG_DATA(nlh), message, len);

    memset(&dest_addr, 0, sizeof(struct sockaddr_nl));
    dest_addr.nl_family = AF_NETLINK;
    dest_addr.nl_pid = pid;
    dest_addr.nl_groups = group;

    if (sendto(sock_fd, nlh, nlh->nlmsg_len, 0, (struct sockaddr *)&dest_addr,
               sizeof(struct sockaddr_nl)) != nlh->nlmsg_len) {
      std::cout << "send error!" << std::endl;
      return -3;
    }
    return 0;
  }

  int netlink_recv_message(int sock_fd, unsigned char *message, int *len) {
    struct nlmsghdr *nlh = NULL;
    char nlh_payload[NLMSG_SPACE(MAX_PAYLOAD)];
    nlh = (struct nlmsghdr *)&nlh_payload[0];
    struct sockaddr_nl src_addr;
    socklen_t addrlen = sizeof(struct sockaddr_nl);

    if (!message || !len) {
      return -1;
    }
    memset(&src_addr, 0, sizeof(struct sockaddr_nl));
    if (recvfrom(sock_fd, nlh, NLMSG_SPACE(MAX_PAYLOAD), 0,
                 (struct sockaddr *)&src_addr, (socklen_t *)&addrlen) < 0) {
      std::cout << "recv no msg, error! sock_fd:" << sock_fd << std::endl;
      return -3;
    }
    *len = nlh->nlmsg_len - NLMSG_SPACE(0);
    memcpy(message, (unsigned char *)NLMSG_DATA(nlh), *len);
    return 0;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_data_publisher");

  IcmKoReader imu;
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
