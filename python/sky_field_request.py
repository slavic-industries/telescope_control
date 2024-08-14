from skyfield.api import load, Topos
from skyfield.data import hipparcos
from skyfield.api import N, E, wgs84
from skyfield.magnitudelib import planetary_magnitude
from skyfield import almanac
import time
import datetime
from pytz import timezone
import pytz

# Create a timescale and ask the current time.
ts = load.timescale()
t = ts.now()

time_zone = timezone('CET')
print(time_zone)
print(type(time_zone))

# Define the observer's location (latitude, longitude)
# observer = Topos('57.311867 N', '11.159813 E')  # Læsø


# Load the JPL ephemeris DE421 (covers 1900-2050).
planets = load('de421.bsp')
earth = planets['earth']
mars = planets['mars']
venus = planets['venus']
target = venus

# Load the Hipparcos star catalog
with load.open(hipparcos.URL) as f:
    stars = hipparcos.load_dataframe(f)

observer = earth + wgs84.latlon(57.31169 * N, 11.14288 * E)

# What's the position of Mars, viewed from Earth?
while 1:
    ts = load.timescale()
    dt = datetime.datetime.now(tz=pytz.timezone('Europe/Warsaw'))
    dt_end = dt + datetime.timedelta(days=1)
    t = ts.now()
    t_end = ts.from_datetime(dt_end)
    astrometric = observer.at(t).observe(target)
    alt, az, d = astrometric.apparent().altaz()
    ra, dec, distance = astrometric.radec()
    mag = planetary_magnitude(astrometric)

    print(f"RA: {ra}\tDEC: {dec}\tMAG: {mag}t\tALT: {alt}\tAZ: {az}")
    time.sleep(1)
