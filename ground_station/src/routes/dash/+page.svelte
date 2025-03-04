<script>
  import Map from "$lib/components/Map.svelte"; // Top Left
  import SerialData from "$lib/components/SerialData.svelte"; // Top Right
  import Charts from "$lib/components/Charts.svelte"; // Bottom Left
  import RocketVisualization from "$lib/components/RocketVisualization.svelte"; // Bottom Right
  import BottomBar from "$lib/components/BottomBar.svelte"; // Bottom Bar
</script>

<div class="grid grid-rows-[1fr_1fr_auto] grid-cols-2 gap-2 h-screen">
    <div class="row-span-1 col-span-1">
        <Map />
    </div>
    <div class="row-span-1 col-span-1">
        <SerialData />
    </div>
    <div class="row-span-1 col-span-1">
        <Charts />
    </div>
    <div class="row-span-1 col-span-1">
        <RocketVisualization />
    </div>
    <div class="row-span-1 col-span-2">
        <BottomBar />
    </div>
</div>

<style lang="postcss">
    @reference "tailwindcss/theme";
</style>