#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from collections import deque
import time

class ScanFilter:
    def __init__(self):
        rospy.init_node('scan_filter')

        self.window_size = rospy.get_param('~window_size', 5)  # окно медианного фильтра, обычно 3 или 5
        self.sma_size = rospy.get_param('~sma_size', 3)        # окно скользящего среднего

        self.ranges_buffer = deque(maxlen=self.sma_size)  # буфер для скользящего среднего

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.filtered_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)

        self.prev_time = None
        self.filtered_count = 0

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Фильтрация NaN, Inf, вне диапазона
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        filtered_ranges = np.where(valid_mask, ranges, np.nan)

        # Добавляем в буфер скользящего среднего
        self.ranges_buffer.append(filtered_ranges)

        # Скользящее среднее
        sma_array = np.nanmean(np.array(self.ranges_buffer), axis=0)

        # Медианный фильтр (окно self.window_size)
        filtered_median = self.median_filter(sma_array, self.window_size)

        # Создаём новое сообщение LaserScan
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Заменяем NaN на 0.0 (либо можно оставить max range, чтобы не сломать)
        clean_ranges = np.where(np.isnan(filtered_median), msg.range_max + 1.0, filtered_median)
        filtered_msg.ranges = clean_ranges.tolist()

        self.filtered_pub.publish(filtered_msg)

        # Частота публикации /scan_filtered
        now = time.time()
        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                freq = 1.0 / dt
                rospy.loginfo_throttle(5, f"Filtered scan publish frequency: {freq:.2f} Hz")
        self.prev_time = now


    def median_filter(self, data, window_size):
        # Простой медианный фильтр по всему массиву с выбранным окном
        half = window_size // 2
        filtered = np.copy(data)

        for i in range(len(data)):
            left = max(0, i - half)
            right = min(len(data), i + half + 1)
            window = data[left:right]
            filtered[i] = np.nanmedian(window)
        return filtered


if __name__ == '__main__':
    try:
        node = ScanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
