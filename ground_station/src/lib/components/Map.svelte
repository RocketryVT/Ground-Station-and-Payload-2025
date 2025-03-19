<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { Deck, MapView, type MapViewState} from 'deck.gl';
  import { TerrainLayer } from '@deck.gl/geo-layers';
  import { TileLayer } from '@deck.gl/geo-layers';
  import { BitmapLayer } from '@deck.gl/layers';
  import { ScenegraphLayer } from '@deck.gl/mesh-layers';

  import { gpsCoordinates } from '$lib/stores';
  let deckgl: Deck<[MapView, MapView]>;

  setInterval(() => {
          gpsCoordinates.update((value) => {
            return {
                  ...value,
                  // longitude: value.longitude + 0.0001,
                  // latitude: value.latitude + 0.0001,
                  altitude: parseFloat((value.altitude + 0.1).toFixed(1))
              };
          });
  }, 10);

  $: gpsCoordinates.subscribe((value) => {
    if (deckgl) {
      deckgl.setProps({
        viewState: {
          'mainView': {
            ...currentViewState.main,
          },
          'miniMapView': {
            ...currentViewState.minimap,
            latitude: $gpsCoordinates.latitude,
            longitude: $gpsCoordinates.longitude,
          }
        },
        layers: [
          ...deckgl.props.layers.filter(layer => layer.id !== 'rocket-layer'),
          new ScenegraphLayer({
            id: 'rocket-layer',
            data: [{ ...gpsCoordinates}],
            pickable: true,
            scenegraph: '/models/rocket-model/Rocket.glb',
            getPosition: (d: { longitude: any; latitude: any; altitude: any }) => [d.longitude, d.latitude, d.altitude],
            getOrientation: [0, 0, 0+90],
            sizeScale: 10
          })
        ],
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
          data: [{ ...gpsCoordinates}],
          pickable: true,
          scenegraph: '/models/rocket-model/Rocket.glb',
          getPosition: (d: { longitude: any; latitude: any; altitude: any}) => [d.longitude, d.latitude, d.altitude],
          getOrientation: [0, 0, 90],
          sizeScale: 10
      });

      deckgl = new Deck({
        canvas: 'deck-canvas',
        controller: true,
        initialViewState: {
          mainView: {
            longitude: $gpsCoordinates.longitude,
            latitude: $gpsCoordinates.latitude,
            zoom: 11.5,
            bearing: 140,
            pitch: 60,
          },
          miniMapView: {
            longitude: $gpsCoordinates.longitude,
            latitude: $gpsCoordinates.latitude,
            zoom: 11.5,
            bearing: 0,
            pitch: 0,
          },
        },
        onViewStateChange,
        useDevicePixels: true,
        layers: [
          osmTileLayer,
          terrainLayer,
          rocketLayer
        ],
        layerFilter: ({layer, viewport}) => {
          if (viewport.id === 'miniMapView' && layer.id === 'terrain' ) {
            return false;
          }
          if (viewport.id === 'miniMapView' && layer.id === 'rocket-layer' ) {
            return false;
          }
          return true;
        },
        views: [
          new MapView({
            id: 'mainView',
            controller: true,
            clear: false
          }),
          new MapView({
            id: 'miniMapView',
            controller: false,
            // Bottom Left
            x: '70%',
            y: '60%',
            width: '30%',
            height: '40%',
            clear: true,
          }),
        ]
      });
});  

onDestroy(() => {
  if (deckgl) deckgl.finalize();
});

let currentViewState: {
  main: MapViewState;
  minimap: MapViewState
} = {
  main: {
    longitude: $gpsCoordinates.longitude,
    latitude: $gpsCoordinates.latitude,
    zoom: 11.5,
  },
  minimap: {
    longitude: $gpsCoordinates.longitude,
    latitude: $gpsCoordinates.latitude,
    zoom: 11.5,
  }
};

function onViewStateChange(params: {
  viewId: string;
  viewState: MapViewState;
}) {
  const { viewId, viewState } = params;
  if (viewId === 'miniMapView') {
    currentViewState.minimap = viewState;
    currentViewState.minimap.latitude = currentViewState.main.latitude;
    currentViewState.minimap.longitude = currentViewState.main.longitude;
  } else if (viewId === 'mainView') {
    currentViewState.main = viewState;
  }
};

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