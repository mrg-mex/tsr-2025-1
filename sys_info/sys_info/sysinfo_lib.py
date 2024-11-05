"""
sysinfo.py

This module provides the SysInfo class, which includes methods to gather various system information
such as CPU usage, memory status, disk space, network interface details, and ROS (Robot Operating System)
environment variables.

Classes:
    - SysInfo: A class with methods for retrieving and managing system information.

Usage example:
    from arrg_utils import SysInfo

    sys_info = SysInfo()
    print(sys_info.get_system_report())
    print(sys_info.get_system_snapshot())
"""

import os
import time
import subprocess
import json
import re
import platform
from typing import List, Dict, Union, Optional


class SysInfo:
    """
    A class to retrieve and manage system information, including host details, disk usage,
    RAM information, CPU statistics, and ROS (Robot Operating System) environment settings.
    """

    def __init__(self) -> None:
        """
        Initializes the SysInfo instance with system command strings and regex patterns.
        """
        self.__ip_info = "ip -j -4 address"
        self.__ip_regex_pattern = (
            r"^((25[0-5]|(2[0-4]|1[0-9]|[1-9]|)[0-9])(\.(?!$)|$)){4}$"
        )
        self.__exclusions = ["lo", "docker0"]
        self.__host_ip = "hostname -I | cut -d ' ' -f1"
        self.__host_name = "hostname -s"
        self.__cpu_stats_info = "grep 'cpu.' /proc/stat | awk '{printf \"%s|%i|%i|%i|%i@\", $1, $2, $3, $4, $5}'"
        self.__cpu_usage_short = "grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage}'"
        self.__sys_date = "date '+%d/%m/%Y|%H:%M:%S'"
        self.__ram_info = (
            "free -h | awk 'NR==2{printf \"%.1f|%.1f|%.1f|%.1f\", $2, $3, $4, $7}'"
        )
        self.__disk_info = 'df -h | awk \'$NF=="/"{printf "%.1f|%.1f|%.1f", $2,$3,$4}\''

    def __execute_command(self, command: str) -> Optional[str]:
        """Executes a shell command and returns its output as a decoded string.

        Args:
            command (str): The shell command to execute.

        Returns:
            Optional[str]: The command output as a decoded string if successful, None otherwise.
        """
        try:
            result = subprocess.check_output(command, shell=True)
            return result.decode().strip()
        except subprocess.CalledProcessError as e:
            print(f"Error executing command '{command}': {e}")
            return None

    def get_host_info(self) -> Dict[str, str]:
        """
        Retrieves the hostname and IP address of the system.

        Returns:
            dict: A dictionary containing `name` and `ip` keys with corresponding values.
        """
        str_ip = self.__execute_command(self.__host_ip) or ""
        host_name = self.__execute_command(self.__host_name) or ""
        return {"name": host_name, "ip": str_ip}

    def __parse_platform(self) -> Dict[str, Union[str, Dict[str, str]]]:
        """
        Retrieves detailed OS information.

        Returns:
            dict: A dictionary containing detailed OS information.
        """
        platform_info = platform.uname()
        additional_info = {}
        match platform_info.system.lower():
            case "windows":
                win32_plat = platform.win32_ver()
                additional_info = {
                    "version": {
                        "release": win32_plat[0],
                        "version": win32_plat[1],
                        "csd": win32_plat[2],
                        "ptype": win32_plat[3],
                    },
                    "edition": platform.win32_edition(),
                    "is_iot": platform.win32_is_iot(),
                }
            case "linux":
                linux_info = platform.freedesktop_os_release()
                if linux_info["ID"] == "ubuntu":
                    additional_info = {
                        "id": linux_info["ID"],
                        "name": linux_info["NAME"],
                        "pretty_name": linux_info["PRETTY_NAME"],
                        "version_id": linux_info["VERSION_ID"],
                        "version": linux_info["VERSION"],
                        "version_codename": linux_info["VERSION_CODENAME"],
                        "id_like": linux_info["ID_LIKE"],
                        "ubuntu_codename": linux_info["UBUNTU_CODENAME"],
                    }
                else:
                    additional_info = linux_info
            case _:
                additional_info = {
                    "message": "No relevant at this moment (yes, that's include mac).",
                }
        return {
            "system": platform_info.system,
            "machine": platform_info.machine,
            "additional_info": additional_info,
        }

    def get_free_disk(self) -> Dict[str, float]:
        """
        Retrieves the total, used, and available disk space on the root directory.

        Returns:
            dict: A dictionary containing `size`, `used`, and `available`
            keys with corresponding values in GB.
        """
        disk_info = self.__execute_command(self.__disk_info)
        if disk_info:
            disk_vals = disk_info.split("|")
            return {
                "size": float(disk_vals[0]),
                "used": float(disk_vals[1]),
                "available": float(disk_vals[2]),
            }

        return {"size": 0.0, "used": 0.0, "available": 0.0}

    def get_free_ram(self) -> Dict[str, float]:
        """
        Retrieves the RAM statistics, including total, used, free, and available memory.

        Returns:
            dict: A dictionary containing `total`, `used`, `free`, and `available` keys.
        """
        ram_info = self.__execute_command(self.__ram_info)
        if ram_info:
            ram_vals = ram_info.split("|")
            return {
                "total": float(ram_vals[0]),
                "used": float(ram_vals[1]),
                "free": float(ram_vals[2]),
                "available": float(ram_vals[3]),
            }

        return {"total": 0.0, "used": 0.0, "free": 0.0, "available": 0.0}

    def get_system_date(self) -> Dict[str, str]:
        """
        Retrieves the current system date and time.

        Returns:
            dict: A dictionary containing `date` and `time` keys.
        """
        date_time = self.__execute_command(self.__sys_date)
        if date_time:
            date_parts = date_time.split("|")
            return {"date": date_parts[0], "time": date_parts[1]}

        return {"date": "", "time": ""}

    def __compute_cpu_usage(
        self, user_val: int, system_val: int, idle_val: int
    ) -> float:
        """
        Computes the CPU usage percentage based on user, system, and idle values.

        Parameters:
            user_val (int): User mode CPU time.
            system_val (int): System mode CPU time.
            idle_val (int): Idle CPU time.

        Returns:
            float: The CPU usage percentage.
        """
        return float(user_val + system_val) * 100 / (user_val + system_val + idle_val)

    def __full_cpu_stats(self) -> List[Dict[str, Union[int, float, str]]]:
        """
        Retrieves detailed CPU statistics for each core.

        Returns:
            list: A list of dictionaries containing detailed CPU statistics for each core.
        """
        cpu_cmd_exec = self.__execute_command(self.__cpu_stats_info)
        cpu_parsed = str(cpu_cmd_exec).lstrip("b'").rstrip("@'")
        cpu_stats = []
        if cpu_cmd_exec:
            cpu_array = cpu_parsed.split("@")
            for indx, cpu_data in enumerate(cpu_array):
                core_info = cpu_data.split("|")
                user_val = int(core_info[1])
                nice_val = int(core_info[2])
                system_val = int(core_info[3])
                idle_val = int(core_info[4])
                perc_usaged = self.__compute_cpu_usage(
                    user_val=user_val, system_val=system_val, idle_val=idle_val
                )
                cpu_type = "total" if indx == 0 else "core"
                cpu_stats.append(
                    {
                        "id": indx,
                        "label": core_info[0],
                        "type": cpu_type,
                        "core": -1 if indx == 0 else (indx - 1),
                        "user": user_val,
                        "nice:": nice_val,
                        "system": system_val,
                        "idle": idle_val,
                        "usaged": perc_usaged,
                    }
                )

        return cpu_stats

    def __short_cpu_stats(self) -> List[Dict[str, Union[int, float, str]]]:
        """
        Retrieves a summary of the overall CPU usage.

        Returns:
            list: A list containing a dictionary with a single CPU usage percentage.
        """
        cpu_usage = self.__execute_command(self.__cpu_usage_short)

        return [{"id": 0, "usaged": float(cpu_usage)}] if cpu_usage else []

    def get_cpu_usage(
        self, compute_value_only=False
    ) -> List[Dict[str, Union[int, float, str]]]:
        """
        Retrieves CPU usage information, either full stats or a summary.

        Parameters:
            compute_value_only (bool): If True, returns a summary of the overall CPU usage.
            Defaults to False.

        Returns:
            list: CPU usage information as a list of dictionaries.
        """

        return (
            self.__short_cpu_stats() if compute_value_only else self.__full_cpu_stats()
        )

    def __validate_ip(self, ipaddress: str) -> bool:
        """
        Validates an IP address using a regex pattern.

        Parameters:
            ipaddress (str): The IP address to validate.

        Returns:
            bool: True if the IP address is valid, False otherwise.
        """
        return bool(re.match(self.__ip_regex_pattern, ipaddress))

    def parse_network_interfaces(
        self, filtered=True, target_ip: str = ""
    ) -> List[Dict[str, Union[str, int, dict]]]:
        """
        Parses and retrieves network interfaces with optional filtering by exclusions and target IP.

        Parameters:
            filtered (bool): If True, excludes interfaces in self.__exclusions. Defaults to True.
            target_ip (str): A target IP address to filter. Defaults to an empty string.

        Returns:
            list: A list of dictionaries with network interface information.
        """
        ip_data = self.__execute_command(self.__ip_info)
        ip_parsed = []
        if ip_data:
            ip_info = json.loads(ip_data)
            for ip_item in ip_info:
                ifname = ip_item["ifname"]
                if not filtered or ifname not in self.__exclusions:
                    localip = ip_item["addr_info"][0]["local"]
                    if target_ip in ("", localip):
                        ip_parsed.append(ip_item)

        return ip_parsed

    def get_ros_info(self) -> Optional[Dict[str, Union[int, str, bool]]]:
        """
        Retrieves the ROS (Robot Operating System) environment details.

        Returns:
            dict or None: A dictionary containing ROS environment variables or None if ROS is not set.
        """
        ros_version = os.environ.get("ROS_VERSION")
        if not ros_version:
            return None

        return {
            "version": int(ros_version),
            "distro": os.environ.get("ROS_DISTRO", ""),
            "domain_id": int(os.environ.get("ROS_DOMAIN_ID", -1)),
            "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY") != "0",
        }

    def get_system_report(self) -> Dict[str, Union[str, dict, list]]:
        """
        Generates a comprehensive system report including:
            host, platform, CPU, RAM, disk, and network information.

        Returns:
            dict: A dictionary containing detailed system information.
        """
        date_time = self.get_system_date()
        host_data = self.get_host_info()
        os_data = self.__parse_platform()
        cpu_stats = self.get_cpu_usage()
        disk_data = self.get_free_disk()
        ram_data = self.get_free_ram()
        network_interfaces = self.parse_network_interfaces()
        ros_env = self.get_ros_info()

        return {
            "host": host_data["name"],
            "platform": os_data,
            "ip": host_data["ip"],
            "date": date_time["date"],
            "time": date_time["time"],
            "cpu_stats": cpu_stats,
            "disk": disk_data,
            "ram": ram_data,
            "network": network_interfaces,
            "ros": ros_env if ros_env else "No ROS enviroment",
        }

    def get_system_snapshot(self) -> Dict[str, Union[str, dict, float]]:
        """
        Generates a snapshot of the system's current CPU, RAM, disk usage, and IP address.

        Returns:
            dict: A dictionary containing a concise system snapshot.
        """
        date_time = self.get_system_date()
        host_data = self.get_host_info()
        cpu_stats = self.get_cpu_usage(compute_value_only=True)
        disk_data = self.get_free_disk()
        ram_data = self.get_free_ram()

        sys_snapshot = {
            "cpu": cpu_stats[0]["usaged"],
            "time": date_time["time"],
            "ram": {
                "available": ram_data["available"],
                "total": ram_data["total"],
            },
            "disk": {
                "available": disk_data["available"],
                "total": disk_data["size"],
            },
            "ip": host_data["ip"],
        }

        return sys_snapshot


def main():
    """
    Main function to print system report and system snapshot every 2 seconds.
    """
    sys_info = SysInfo()
    print(sys_info.get_system_report())
    try:
        while True:
            print(sys_info.get_system_snapshot())
            time.sleep(2)
    except KeyboardInterrupt:
        print(" Program closed! ")


if __name__ == "__main__":
    main()
