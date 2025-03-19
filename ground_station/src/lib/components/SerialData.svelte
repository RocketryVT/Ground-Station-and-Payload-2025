<script lang="ts">
    export let data: any[] = [];

    import { writable } from 'svelte/store';

    // interface Coordinates {
    //     longitude: number;
    //     latitude: number;
    // }

    // const gpsCoordinates = writable<Coordinates[]>([]);

    // Example of adding a new GPS coordinate
    // gpsCoordinates.update(coords => [...coords, { longitude: -122.45, latitude: 37.78 }]);
</script>
  
  <style>
    .serial-data {
      overflow-y: auto;
      height: 100%;
    }
  </style>
  
  <div class="serial-data">
    {#each data as item}
      <div>{item}</div>
    {/each}
  </div>