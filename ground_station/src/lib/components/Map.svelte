<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Deck, MapView} from 'deck.gl';
  import { TerrainLayer } from '@deck.gl/geo-layers';
  import { TileLayer } from '@deck.gl/geo-layers';
  // import { OrbitView } from '@deck.gl/core';
  import { BitmapLayer } from '@deck.gl/layers';
  import { ScenegraphLayer } from '@deck.gl/mesh-layers';
  import { writable } from 'svelte/store';


  // Store for GPS coordinates
  export let gpsCoordinates = writable([]);
  export let rocketAltitude = writable(0);

  // /** @type {import('maplibre-gl').Map} */
  // let map: Map;
  /** @type {import('deck.gl').Deck<MapView>} */
  let deck: Deck<MapView>;

  const initialCoordinate = { longitude: -122.20, latitude: 46.20, altitude: 0 };

  $: gpsCoordinates.subscribe((coords: {altitude: number; longitude: number; latitude: number; }[]) => {
      if (coords.length > 0) {
          const latestCoordinate = coords[coords.length - 1];
          rocketAltitude.update(altitude => latestCoordinate.altitude || altitude);
          deck.setProps({
              viewState: {
                  longitude: latestCoordinate.longitude,
                  latitude: latestCoordinate.latitude,
                  zoom: 11.5,
                  bearing: 140,
                  pitch: 60
              },
              layers: [
                  ...deck.props.layers,
                  new ScenegraphLayer({
                      id: 'rocket-layer',
                      data: [{ ...latestCoordinate, altitude: $rocketAltitude }],
                      pickable: true,
                      scenegraph: '/models/rocket-model/Rocket.glb',
                      getPosition: (d: { longitude: any; latitude: any; altitude: any }) => [d.longitude, d.latitude, d.altitude],
                      getOrientation: [0, 0, 0+90],
                      sizeScale: 10
                  })
              ]
          });
      }
  });

onMount(() => {
      const key = 'AZogTcU2i1A2d5jt4cJx';

      const TERRAIN_IMAGE = `https://api.maptiler.com/tiles/terrain-rgb-v2/{z}/{x}/{y}.webp?key=${key}`;
      const SURFACE_IMAGE = `https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key=${key}`;

      const ELEVATION_DECODER = {
        rScaler: 6553.6,
        gScaler: 25.6,
        bScaler: 0.1,
        offset: -10000
      };

      const osmTileLayer = new TileLayer({
          id: 'osm-tile-layer',
          data: 'https://c.tile.openstreetmap.org/{z}/{x}/{y}.png',
          minZoom: 0,
          maxZoom: 19,
          tileSize: 256,
          maxRequests: 20,
          maxCacheSize: 1000,
          renderSubLayers: props => {
            const {boundingBox} = props.tile;

            return new BitmapLayer(props, {
              data: undefined,
              image: props.data,
              bounds: [boundingBox[0][0], boundingBox[0][1], boundingBox[1][0], boundingBox[1][1]]
            });
          },
      });

      const terrainLayer = new TerrainLayer({
          id: 'terrain',
          minZoom: 0,
          maxZoom: 12,
          elevationDecoder: ELEVATION_DECODER,
          elevationData: TERRAIN_IMAGE,
          texture: SURFACE_IMAGE,
          wireframe: false,
          color: [255, 255, 255],
          maxCacheSize: 1000,
      });

      const rocketLayer = new ScenegraphLayer({
          id: 'rocket-layer',
          data: [initialCoordinate],
          pickable: true,
          scenegraph: '/models/rocket-model/Rocket.glb',
          getPosition: (d: { longitude: any; latitude: any; altitude: any}) => [d.longitude, d.latitude, d.altitude],
          getOrientation: [0, 0, 90],
          sizeScale: 10
      });

      deck = new Deck({
          canvas: 'deck-canvas',
          initialViewState: {
            latitude: 46.20,
            longitude: -122.20,
            zoom: 11.5,
            bearing: 140,
            pitch: 60,
            maxPitch: 89
          },
          useDevicePixels: false,
          controller: true,
          layers: [osmTileLayer, terrainLayer, rocketLayer],
          views: new MapView({repeat: false}),
          // layers: [rocketLayer],
      });

      const interval = setInterval(() => {
          rocketAltitude.update(altitude => altitude + 1);
          deck.setProps({
            //   viewState: {
            //       longitude: initialCoordinate.longitude,
            //       latitude: initialCoordinate.latitude,
            //       zoom: 15,
            //       bearing: 140,
            //       pitch: 60
            //   },
              layers: [
                    ...deck.props.layers.filter(layer => layer.id !== 'rocket-layer'),
                  new ScenegraphLayer({
                      id: 'rocket-layer',
                      data: [{ ...initialCoordinate, altitude: $rocketAltitude }],
                      pickable: true,
                      scenegraph: '/models/rocket-model/Rocket.glb',
                      getPosition: (d: { longitude: any; latitude: any; altitude: any }) => [d.longitude, d.latitude, d.altitude],
                      getOrientation: [0, 0, 0+90],
                      sizeScale: 10
                  })
              ]
          });
      }, 10);
});  

onDestroy(() => {
      if (deck) deck.finalize();
});

</script>

<style>
  .map-container {
      position: relative;
      width: 100%;
      height: 100%;
  }
  #deck-canvas {
      position: relative;
      width: 100%;
      height: 100%;
  }
</style>

<div class="map-container">
  <canvas id="deck-canvas"></canvas>
</div>