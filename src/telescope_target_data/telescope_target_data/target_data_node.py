import rclpy
from rclpy.node import Node
from telescope_interfaces.srv import RequestTargetData
from telescope_interfaces.srv import SetObserver
from telescope_interfaces.msg import TargetCoordinates

import astropy.units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord
from astropy.coordinates import EarthLocation
from astropy.coordinates import AltAz
from astropy.coordinates import get_body
from astropy.coordinates import solar_system_ephemeris


class TargetData(Node):

    def __init__(self):
        super().__init__('target_data')

        self.publisher_target_alt_az = self.create_publisher(
            TargetCoordinates, 'target_coordinates', 10)     # CHANGE

        timer_period = 0.02
        self.timer_target_alt_az = self.create_timer(
            timer_period, self.target_alt_az_callback)

        self.service_request_target_data = self.create_service(
            RequestTargetData,
            "request_target_data",
            self.request_target_data_callback
        )

        self.service_set_observer = self.create_service(
            SetObserver,
            "set_observer",
            self.set_observer_callback
        )

        self.observer_lattitude = 0.0
        self.observer_longitude = 0.0
        self.observer_elevation = 0.0
        self.observer_pressure = 0.0
        self.observer_temperature = 0.0
        self.observer_rel_humid = 0.0
        self.observer_timezone = "UTC"

        self.observer = EarthLocation.from_geodetic(self.observer_longitude,
                                                    self.observer_lattitude,
                                                    self.observer_elevation)

        self.target_body_info = SkyCoord(0.0, 0.0, unit=u.deg, frame='icrs')

        self.target_name = "None"
        self.target_ra = 0.0
        self.target_dec = 0.0
        self.target_distance = 0.0
        self.target_alt = 0.0
        self.target_az = 0.0

        self.get_logger().info(f"'{self.get_name()}' node created")

    def target_alt_az_callback(self):
        msg = TargetCoordinates()
        if self.target_name != "None":
            t = Time.now()

            self.target_body_info = get_body(
                self.target_name, t, self.observer)
            altaz_frame = AltAz(obstime=t,
                                location=self.observer)
            altaz_coor = self.target_body_info.transform_to(altaz_frame)

            msg.alt = float(altaz_coor.alt.deg)
            msg.az = float(altaz_coor.az.deg)
            msg.ra = float(self.target_body_info.ra.hour)
            msg.dec = float(self.target_body_info.dec.deg)
            msg.alt_deg = altaz_coor.alt.to_string(
                unit=u.deg, sep=':', pad=True)
            msg.az_deg = altaz_coor.az.to_string(unit=u.deg, sep=':', pad=True)

            self.publisher_target_alt_az.publish(msg)
            # self.get_logger().info(f"Az: {msg.az_deg}\tAlt: {msg.alt_deg}")

    def request_target_data_callback(self, request, response):
        self.target_name = request.target_name
        response.success = self._get_target_info()
        response.target_name = self.target_name
        response.dist = self.target_body_info.distance.to(u.m).value
        self.get_logger().info(
            f"service: 'request_target_data': request.target_name='{request.target_name}'")
        return response

    def set_observer_callback(self, request, response):
        self.observer_lattitude = float(request.lat)
        self.observer_longitude = float(request.lon)
        self.observer_elevation = float(request.alt)
        self.observer_pressure = float(request.press)
        self.observer_temperature = float(request.temp)
        self.observer_rel_humid = float(request.rel_humid)
        self.observer_timezone = request.time_zone
        self.observer = EarthLocation.from_geodetic(self.observer_longitude,
                                                    self.observer_lattitude,
                                                    self.observer_elevation)
        response.success = True
        self.get_logger().info("service: 'set_observer'")
        return response

    def _get_target_info(self):
        with solar_system_ephemeris.set('builtin'):
            t = Time.now()
            # self.observer = EarthLocation.from_geodetic(self.observer_longitude,
            #                                             self.observer_lattitude,
            #                                             self.observer_elevation)
            try:
                self.target_body_info = get_body(
                    self.target_name, t, self.observer)
                altaz_frame = AltAz(obstime=t,
                                    location=self.observer)
                altaz_coor = self.target_body_info.transform_to(altaz_frame)
                self.target_alt = altaz_coor.alt.deg
                self.target_az = altaz_coor.az.deg

                return True
            except Exception as e:
                self.get_logger().error(
                    f"Unable to get target info for {self.target_name}: {e}")
                return False


def main(args=None):
    rclpy.init(args=args)
    target_data = TargetData()
    rclpy.spin(target_data)
    target_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
