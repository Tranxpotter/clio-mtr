#!/usr/bin/env python3

import sqlite3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSqliteLogger(Node):
    def __init__(self):
        super().__init__('odom_sqlite_logger')

        # Declare parameters
        self.declare_parameter('db_path', 'inspections.db')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('is_gt', False)

        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.is_gt = self.get_parameter('is_gt').get_parameter_value().bool_value

        # Initialize SQLite database
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        self._init_db()

        # Create a new inspection run record
        self.inspection_id = self._create_inspection_record()

        # Subscribe to Odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f"Logging '{self.odom_topic}' -> '{self.db_path}' "
            f"[Inspection ID: {self.inspection_id}, Ground Truth: {self.is_gt}]"
        )

    def _init_db(self):
        """Creates tables if they do not exist."""
        # started_at defaults to yyyy-mm-dd HH:MM:SS format via datetime('now')
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS inspections (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at TEXT NOT NULL DEFAULT (datetime('now')),
                is_gt BOOL NOT NULL
            )
        ''')

        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS images (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                inspection_id INTEGER NOT NULL,
                timestamp_ns INTEGER NOT NULL,
                tf_translation_x REAL NOT NULL,
                tf_translation_y REAL NOT NULL,
                tf_translation_z REAL NOT NULL,
                tf_rotation_x REAL NOT NULL,
                tf_rotation_y REAL NOT NULL,
                tf_rotation_z REAL NOT NULL,
                tf_rotation_w REAL NOT NULL,
                filename TEXT NOT NULL,
                FOREIGN KEY (inspection_id) REFERENCES inspections (id)
            )
        ''')
        self.conn.commit()

    def _create_inspection_record(self) -> int:
        """Inserts inspection run record letting SQLite set default started_at timestamp."""
        self.cursor.execute(
            'INSERT INTO inspections (is_gt) VALUES (?)',
            (self.is_gt,)
        )
        self.conn.commit()
        return self.cursor.lastrowid

    def odom_callback(self, msg: Odometry):
        """Extracts message header stamp and pose to insert into images table."""
        # Convert header timestamp (sec, nanosec) into total nanoseconds
        timestamp_ns = (msg.header.stamp.sec * 1_000_000_000) + msg.header.stamp.nanosec

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Step 1: Insert data with temporary placeholder filename
        self.cursor.execute('''
            INSERT INTO images (
                inspection_id, timestamp_ns,
                tf_translation_x, tf_translation_y, tf_translation_z,
                tf_rotation_x, tf_rotation_y, tf_rotation_z, tf_rotation_w,
                filename
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            self.inspection_id, timestamp_ns,
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
            ""
        ))

        # Step 2: Retrieve row ID and update filename to <id>.jpg
        image_id = self.cursor.lastrowid
        filename = f"{image_id}.jpg"

        self.cursor.execute('UPDATE images SET filename = ? WHERE id = ?', (filename, image_id))
        self.conn.commit()

    def destroy_node(self):
        if hasattr(self, 'conn') and self.conn:
            self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomSqliteLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()