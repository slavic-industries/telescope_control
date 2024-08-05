import matplotlib.pyplot as plt
import numpy as np

import astropy

from astropy import units as u
from astropy.coordinates import SkyCoord, Distance
from astropy.io import fits
from astropy.table import QTable
from astropy.utils.data import download_file

from astroquery.gaia import Gaia
Gaia.ROW_LIMIT = 10000  # Set the row limit for returned data

ngc188_center = SkyCoord(12.11*u.deg, 85.26*u.deg, frame='icrs')
print(ngc188_center)

SkyCoord('00h48m26.4s', '85d15m36s', frame='icrs')
print(ngc188_center)
print(f"{ngc188_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {ngc188_center.dec}")

m31_center = SkyCoord.from_name('M31')
print(f"{m31_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {m31_center.dec}")

coordinates = astropy.coordinates.get_icrs_coordinates(name='M 31')
print(f"{coordinates.ra}")

m31_center = SkyCoord(coordinates, frame='icrs')
print(f"{m31_center.ra.to_string(unit=u.hourangle, sep=':', pad=True)}\t {m31_center.dec}")
