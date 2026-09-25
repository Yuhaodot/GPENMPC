# Planning data and external sources

## Overture geographic data

The building and road geometries in `data/cities` derive from Overture Maps
Buildings and Transportation, release **2026-05-20.0**, downloaded on
**12 June 2026**. Both themes are licensed under
[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/), as listed in the
[official Overture attribution information](https://docs.overturemaps.org/attribution/).

**© OpenStreetMap contributors, Overture Maps Foundation.** The geographic
data include contributions from the following sources:

| Source | Where used | License / source information |
| --- | --- | --- |
| OpenStreetMap contributors | Buildings and roads in all three cities | [ODbL 1.0](https://www.openstreetmap.org/copyright) |
| Microsoft ML Buildings | Buildings in all three cities | ODbL 1.0 in the source records |
| TomTom | Road records in Cambridge and Seattle | ODbL 1.0 in the source records |
| Esri Community Maps contributors | Five retained building records in Seattle | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), with OpenStreetMap waivers recorded in the source metadata |
| U.S. Geological Survey, USGS Lidar | Building-height source records in all three cities | The records have no separate license value; [USGS copyright policy](https://www.usgs.gov/information-policies-and-instructions/copyrights-and-credits) identifies USGS-produced data as public domain and requests source credit |

The map files store source feature identifiers, coordinate transforms and
checksums linking them to the original GeoParquet data.

The study selects a planning window, transforms the public geometry into a local
frame, generates route candidates, and defines depot and delivery locations.
The geographic-data license remains separate from the project-code license.

## M600 public-reference profile

`assets/m600_profile.json` contains platform parameters and a phase-average
power model based on the following sources:

- [DOE/OSTI record 2583936](https://www.osti.gov/pages/servlets/purl/2583936),
  INL/JOU-22-67807-Revision-0, DOI `10.1016/j.eswa.2024.124172`, Table B.1 for
  phase-average power; Elsevier article manuscript hosted by OSTI.
- [DJI Matrice 600 Pro User Manual, version 1.0](https://dl.djicdn.com/downloads/m600%20pro/1208EN/Matrice_600_Pro_User_Manual_v1.0_EN_1208.pdf),
  April 2018, for platform specifications; copyright DJI.

The planning model uses phase-average power for task-energy accounting.
