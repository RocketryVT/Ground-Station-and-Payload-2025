import { writable } from "svelte/store";

export const mach = writable(0.0);
export const velocity = writable(0.0);

interface Coordinates {
    longitude: number;
    latitude: number;
    altitude: number;
}

export const gpsCoordinates = writable<Coordinates>({ longitude: -122.20, latitude: 46.20, altitude: 0.0 });