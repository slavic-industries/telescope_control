from astroplan import FixedTarget
from astropy.coordinates import SkyCoord
from astroplan import Observer
from pytz import timezone
import sys
from astropy.coordinates import EarthLocation
import astropy.units as u
from astropy.coordinates import get_body_barycentric, get_body
from astropy.coordinates import solar_system_ephemeris, EarthLocation
from astropy.time import Time
import matplotlib.pyplot as plt
import numpy as np

import astropy

from astropy import units as u
from astropy.coordinates import SkyCoord, Distance
from astropy.io import fits
from astropy.table import QTable
from astropy.utils.data import download_file

from astroquery.gaia import Gaia


# Gaia.ROW_LIMIT = 10000  # Set the row limit for returned data

# ngc188_center = SkyCoord(12.11*u.deg, 85.26*u.deg, frame='icrs')
# print(ngc188_center)

# SkyCoord('00h48m26.4s', '85d15m36s', frame='icrs')
# print(ngc188_center)
# print(f"{ngc188_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {ngc188_center.dec}")

# m31_center = SkyCoord.from_name('M31')
# print(f"{m31_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {m31_center.dec}")

# coordinates = astropy.coordinates.get_icrs_coordinates(name='M 31')
# print(f"{coordinates.ra}")

# m31_center = SkyCoord(coordinates, frame='icrs')
# print(f"{m31_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {m31_center.dec}")


t = Time.now()
longitude = 56.949191  # '-155d28m48.900s'
latitude = 9.866358  # '+19d49m42.600s'
elevation = 4163 * u.m
location = EarthLocation.from_geodetic(longitude, latitude, elevation)


# with solar_system_ephemeris.set('de432s'):
with solar_system_ephemeris.set('builtin'):
    jup = get_body('jupiter', t, location)
print(jup)
print(f"{jup.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {jup.dec.to_string(unit=u.deg, sep=':', pad=True)}")

sys.exit()
observer = Observer(name='Subaru Telescope',
                    location=location,
                    pressure=0.615 * u.bar,
                    relative_humidity=0.11,
                    temperature=0 * u.deg_C,
                    timezone=timezone('CET'),
                    description="Subaru Telescope on Maunakea, Hawaii")
print(observer)

time = Time.now()

altair = FixedTarget.from_name('Altair')
vega = FixedTarget.from_name('Vega')

coordinates = SkyCoord('20h41m25.9s', '+45d16m49.3s', frame='icrs')
deneb = FixedTarget(name='Deneb', coord=coordinates)


print(observer.target_is_up(time, altair))

print(observer.target_is_up(time, vega))

print(observer.target_is_up(time, deneb))
