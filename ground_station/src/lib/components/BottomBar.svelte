<script>    
    import { mach, velocity, gpsCoordinates } from "$lib/stores";
    let max_mach = 0;

    $: if ($mach > max_mach) {
        max_mach = $mach;
    }
</script>

<style>
    .fixed-width {
        /* width: 100%; */
        /* max-width: 100%; */
        min-width: 20%;
        text-align: center;
    }
    .monospace {
        font-family: 'Courier New', Courier, monospace;
    }
</style>
  
<div class="flex justify-around items-center h-full bg-gray-200">
    <div class="text-center text-black fixed-width">
        <div>Mach</div>
        <div class="text-2xl font-bold monospace">{$mach} Mach</div>
    </div>
    <div class="text-center text-black fixed-width">
        <div>Max Mach</div>
        <div class="text-2xl font-bold monospace">{max_mach} Mach</div>
    </div>
    <div class="text-center text-black fixed-width">
        <div>Velocity</div>
        <div class="text-2xl font-bold monospace">{$velocity} m/s</div>
    </div>
    <div class="text-center text-black fixed-width">
        <div>Altitude</div>
        <div class="text-2xl font-bold monospace">{$gpsCoordinates.altitude} m</div>
    </div>
</div>