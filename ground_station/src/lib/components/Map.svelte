<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Map } from 'maplibre-gl';
  import 'maplibre-gl/dist/maplibre-gl.css';
  import { Deck } from 'deck.gl';
  import {TerrainLayer} from '@deck.gl/geo-layers';
  import { MapView } from 'deck.gl';
  import { PathLayer } from 'deck.gl';
  import { writable } from 'svelte/store';

  // Store for GPS coordinates
  export let gpsCoordinates = writable([]);

  /** @type {import('maplibre-gl').Map} */
  let map: Map;
  /** @type {import('deck.gl').Deck} */
  let deck: Deck;

  $: gpsCoordinates.subscribe((coords: { longitude: number; latitude: number; }[]) => {
      if (coords.length > 0) {
          /**
         * @type {{ longitude: number; latitude: number; }}
         */
          const latestCoordinate = coords[coords.length - 1];
          map.setCenter([latestCoordinate.longitude, latestCoordinate.latitude]);
        //   deck.setProps({
        //       layers: [
        //         //   new PathLayer({
        //         //       id: 'path-layer',
        //         //       data: [{ path: coords.map(coord => [coord.longitude, coord.latitude]) }],
        //         //       getWidth: 5,
        //         //       getColor: [255, 140, 0],
        //         //       widthMinPixels: 2
        //         //   })
        //       ]
        //   });
      }
  });

const layer = new TerrainLayer({
    elevationDecoder: {
        rScaler: 2,
        gScaler: 0,
        bScaler: 0,
        offset: 0
    },
    // Digital elevation model from https://www.usgs.gov/
    elevationData: 'https://raw.githubusercontent.com/visgl/deck.gl-data/master/website/terrain.png',
    texture: 'https://raw.githubusercontent.com/visgl/deck.gl-data/master/website/terrain-mask.png',
    bounds: [-122.5233, 37.6493, -122.3566, 37.8159],
});

  onMount(() => {
    //   map = new Map({
    //       container: 'map',
    //       style: 'https://demotiles.maplibre.org/style.json',
    //       center: [0, 0],
    //       zoom: 2
    //   });

      deck = new Deck({
        //   canvas: 'deck-canvas',
          initialViewState: {
            longitude: -122.4,
            latitude: 37.74,
            zoom: 11
          },
          controller: true,
          layers: [layer]
      });

      map.on('move', () => {
          const { lng, lat } = map.getCenter();
          deck.setProps({
              viewState: {
                  longitude: lng,
                  latitude: lat,
                  zoom: map.getZoom(),
                  bearing: map.getBearing(),
                  pitch: map.getPitch()
              }
          });
      });
  });

  onDestroy(() => {
      if (map) map.remove();
      if (deck) deck.finalize();
  });
</script>

<style>
  #map {
      width: 100%;
      height: 100%;
  }

  #deck-canvas {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
  }
</style>

<div id="map"></div>
<canvas id="deck-canvas"></canvas>