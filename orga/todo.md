

- nächstes Treffen: morgen 16Uhr (14Uhr treffen für anderes seminar)

- pynn anschauen
- braiterberg brain bauen

- das dinge ausprobieren: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/notebooks/HelloPyBullet.ipyn

- jupyter zeug
  - docker?
  - jupyter installieren
  - HelloPyBullet.ipynb anschauen
  - zu DIRECT umbauen
  - Bild rausholen

## done:

- pybullet installiert
- model-datein aus nrp rauskopiert
- zeug runterladen
- local ausführen
- husky urdf suchen und finden
- raum neubauen 
- lib ersetzen
- sensoren einbauen
  - http://wiki.ros.org/urdf/XML/sensor

- brain einbauen
- sensor-brain loop
- brain-motor loop
- full loop

## notizen

> DIRECT mode does allow rendering of images using the built-in software renderer through the 'getCameraImage' API. This can be useful for running simulations in the cloud on servers without GPU.

- bullet3/examples/pybullet/notebooks/HelloPyBullet.ipynb

## versucht und gescheitert

### virtual_room.sdl in pybullet

probleme:

- es fehlen einen haufen Models, die pybullet nicht kennt.
- die models nach pybullet_data zu kopieren hat nicht gebracht: Sie wurden immernoch nicht gefunden
- die uri der meshes umzubiegen hat nicht geklappt; Sie wurden immer noch nicht gefunden (Und ich habe alle möglichen relativen und absoluten pfade probiert)
- die meshes durch würfel zu ersetzen dauert zu lange, es werden hunderte davon in der Datei referenziert
