// Internationalization (i18n) and Accessibility Language Dictionary.
// Milestone 2.0 — Validated Training Platform.

export const TRANSLATIONS = {
  en: {
    depth: 'Depth',
    altitude: 'Altitude',
    heading: 'Heading',
    speed: 'Speed',
    current: 'Current',
    tether: 'Tether',
    flightScore: 'Flight Score',
    grade: 'Grade',
    waypointProgress: 'Waypoint Progress',
    infractions: 'Infractions',
    recTelemetry: 'Rec Telemetry',
    stopSaveCsv: 'Stop & Save CSV',
    faultNone: 'Fault: None',
    instructorBtn: 'Instructor',
    debriefBtn: 'Debrief Report',
    scenarioFree: 'Scenario: Free',
    pilotInput: 'Pilot input',
    keyboardControl: 'Keyboard control',
    assistHold: 'Depth + attitude hold',
    pause: 'Pause',
    resume: 'Resume',
    resetVehicle: 'Reset vehicle',
    stationKeep: 'Station keep',
    autoHeading: 'Auto heading',
    autoDepth: 'Auto depth',
    tetherOk: 'Tether OK',
    gamepad: 'Gamepad',
  },
  no: {
    depth: 'Dybde',
    altitude: 'Høyde over bunn',
    heading: 'Kurs',
    speed: 'Fart',
    current: 'Strøm',
    tether: 'Navlestreng',
    flightScore: 'Flygeskår',
    grade: 'Karakter',
    waypointProgress: 'Rammepunkter',
    infractions: 'Avvik',
    recTelemetry: 'Ta opp telemetri',
    stopSaveCsv: 'Stopp & lagre CSV',
    faultNone: 'Feil: Ingen',
    instructorBtn: 'Instruktør',
    debriefBtn: 'Debrief-rapport',
    scenarioFree: 'Scenario: Fri flyging',
    pilotInput: 'Styreinndata',
    keyboardControl: 'Tastaturkontroll',
    assistHold: 'Dybde- og kurslås',
    pause: 'Pause',
    resume: 'Fortsett',
    resetVehicle: 'Tilbakestill farkost',
    stationKeep: 'Posisjonshold',
    autoHeading: 'Autokurs',
    autoDepth: 'Autodybde',
    tetherOk: 'Kabel OK',
    gamepad: 'Styrespak',
  },
};

let currentLang = 'en';

export function getLanguage() {
  return currentLang;
}

export function setLanguage(lang) {
  if (TRANSLATIONS[lang]) {
    currentLang = lang;
  }
  return currentLang;
}

export function t(key, lang = currentLang) {
  return TRANSLATIONS[lang]?.[key] ?? TRANSLATIONS['en']?.[key] ?? key;
}
