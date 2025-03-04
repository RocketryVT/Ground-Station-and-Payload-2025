<script>
    export let speed = 0;
    export let velocity = 0;
    export let altitude = 0;
</script>
  
<div class="flex justify-around items-center h-full bg-gray-200">
    <div class="text-center text-black">
        <div>Speed</div>
        <div class="text-2xl font-bold">{speed} Mach</div>
    </div>
    <div class="text-center text-black">
        <div>Velocity</div>
        <div class="text-2xl font-bold">{velocity} m/s</div>
    </div>
    <div class="text-center text-black">
        <div>Altitude</div>
        <div class="text-2xl font-bold">{altitude} m</div>
    </div>
</div>