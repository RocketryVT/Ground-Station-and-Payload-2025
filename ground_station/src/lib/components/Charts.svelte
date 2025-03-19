<script lang="ts">
    import { onDestroy, onMount } from 'svelte';
    import * as echarts from 'echarts';
    import { gpsCoordinates } from '$lib/stores';
  
    let myChart: echarts.ECharts;
    let option: echarts.EChartsOption;
    interface DataItem {
        name: string;
        value: [string, number];
    }

    let data: DataItem[] = [];
    let now = new Date();
  
    let unsubscribe = gpsCoordinates.subscribe((coords) => {
        if (coords) {
            now = new Date();
            data.push({
                name: now.toISOString(),
                value: [now.toISOString(), coords.altitude]
            });
        }
    });
  
    option = {
        tooltip: {
            trigger: 'axis',
            formatter: function (params: any) {
                params = params[0];
                let date = new Date(params.name);
                return (
                    date.getHours().toString().padStart(2, '0') + ':' +
                    date.getMinutes().toString().padStart(2, '0') + ':' +
                    date.getSeconds().toString().padStart(2, '0') + '.' +
                    date.getMilliseconds().toString().padStart(3, '0') + ' : ' +
                    params.value[1]
                );
            },
            axisPointer: {
                animation: false
            }
        },
        xAxis: {
            type: 'time',
            axisLabel: {
                formatter: (value: number) => {
                    let date = new Date(value);
                    return (
                        date.getHours().toString().padStart(2, '0') + ':' +
                        date.getMinutes().toString().padStart(2, '0') + ':' +
                        date.getSeconds().toString().padStart(2, '0')
                    );
                },
                hideOverlap: true
            },
            splitLine: { show: false }
        },
        yAxis: {
            type: 'value',
            name: 'Altitude',
            boundaryGap: [0, '10%'],
            splitLine: { show: false }
        },
        series: [
            {
                name: 'GPS Altitude',
                type: 'line',
                showSymbol: false,
                data: data,
                smooth: true
            }
        ],
        grid: {
            containLabel: true,
            left: 10,
            right: 10,
            top: 30,
            bottom: 0
        }
    };

    onMount(() => {
        myChart = echarts.init(document.querySelector('.chart-container') as HTMLDivElement);
        myChart.setOption(option);
        window.addEventListener('resize', handleResize);

        onDestroy(() => {
            window.removeEventListener('resize', handleResize);
            unsubscribe();
        });
    });

    function handleResize() {
        if (myChart) {
            myChart.resize();
        }
    }
    
    setInterval(() => {
        now = new Date();
        data.push({
            name: now.toISOString(),
            value: [now.toISOString(), $gpsCoordinates.altitude]
        })

        if (data.length > 1000) {
            data.shift();
        }
    
        myChart.setOption<echarts.EChartsOption>({
            series: [{ data: data }]
        });
    }, 500);
</script>

<style>
    .chart-container {
        position: relative;
        width: 100%;
        height: 100%;
        background-color: whitesmoke;
        overflow: hidden;
    }
</style>

<div class="chart-container"></div>
