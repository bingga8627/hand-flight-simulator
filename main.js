"use strict";
// AERONAUT — 프레임워크 없이 실행되는 Canvas / MediaPipe 비행 프로토타입.
// CDN은 카메라 연결 시에만 불러옵니다. 로딩 실패해도 비행 화면은 유지됩니다.
const MEDIAPIPE_VERSION = "0.10.22-rc.20250304";
const VISION_URL = `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@${MEDIAPIPE_VERSION}/vision_bundle.mjs`;
const WASM_URL = `https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@${MEDIAPIPE_VERSION}/wasm`;
const MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task";

// 주요 조정값: 좌표는 0~1, 각도는 도, 속도는 초 단위입니다.
const PINCH_THRESHOLD = 0.30; // 엄지-검지 거리 / 손바닥 폭. 카메라 거리에 덜 민감합니다.
const PINCH_RELEASE_THRESHOLD = 0.43; // 해제 문턱을 넓혀 경계에서 깜빡임 방지
const SMOOTHING_FACTOR = 0.30; // 30fps 기준 EMA. 낮을수록 부드럽지만 느립니다.
const STICK_RANGE = 0.19; // 잡은 지점으로부터 이만큼 이동하면 최대 입력
const DEPTH_RANGE = 0.08; // 손바닥 크기의 로그 비율이 약 8% 변하면 최대 Pitch 입력
const DEPTH_DEADZONE = 0.005; // 기준 거리 근처 0.5% 변화만 손 떨림으로 무시
const DEPTH_SMOOTHING = 0.28;
const DEPTH_SHAPE_TOLERANCE = 0.50; // 손바닥 형태 비율이 크게 달라지면 거리 판단 보류
const VERTICAL_PITCH_RANGE = 0.10; // 이지 모드 최대 Pitch까지 필요한 세로 이동량
const HAND_EDGE_MARGIN = 0.12; // 프레임 가장자리 접근 안내 기준
const VERTICAL_PITCH_ASSIST = 0.65; // 거리 조종과 함께 적용되는 위아래 보조 비율
const STICK_CENTER = { x: 0.5, y: 0.79 };
const STICK_VISUAL_RADIUS = 0.115; // 조종 입력을 보여주는 표시 크기(잡기 범위가 아님)
const STICK_RETURN_SPEED = 4.5;
const ROLL_SPEED = 2.1;
const PITCH_SPEED = 3.0;
const MAX_ROLL = 55;
const MAX_PITCH = 24;
const CONTROL_RESPONSE_PRESETS = {
  smooth: { gain:0.85, inputSpeed:5, rollSpeed:ROLL_SPEED, pitchSpeed:PITCH_SPEED },
  normal: { gain:1.0, inputSpeed:8, rollSpeed:3.8, pitchSpeed:4.6 },
  fast: { gain:1.22, inputSpeed:12, rollSpeed:5.8, pitchSpeed:6.6 }
};
const THROTTLE_TOP = 0.22;
const THROTTLE_BOTTOM = 0.82;
const TRACKING_FPS = 24; // 동기식 추론 횟수를 제한하여 Canvas 렌더링 부담 완화
const HAND_LOST_TIMEOUT = 750; // 당기는 동안 손가락이 겹쳐 검출이 잠깐 끊겨도 조종간을 유지
const MAX_DELTA_TIME = 0.05; // 탭 복귀 시 물리값 급변 방지
const CDN_TIMEOUT = 30000;
const CALIBRATION_PREPARE_MS = 2000; // 버튼에서 손을 떼고 자세를 잡을 여유
const CALIBRATION_HOLD_MS = 1500; // 연속으로 안정된 손을 보여줘야 하는 시간
const CALIBRATION_TIMEOUT_MS = 20000;
const CALIBRATION_STABILITY = 0.045; // 기준 샘플로부터 허용하는 정규화 이동 거리
// 이착륙은 실제 항공 규정이 아닌 이 프로토타입의 훈련 판정값입니다.
const RUNWAY = { length: 2200, width: 60, heading: 300, rotationSpeed: 70, rotationPitch: 4, maxLandingSpeed: 130, minLandingSpeed: 45, maxSink: 12, maxBank: 12, maxHeadingError: 18 };
let MISSION_CHECKPOINTS = [
  // 활주로 앞쪽의 가까운 구역에서 짧게 좌우 이동하는 연속 코스입니다.
  { x:0, z:500, altitude:180 },
  { x:70, z:900, altitude:260 },
  { x:-30, z:1300, altitude:340 },
  { x:60, z:1700, altitude:300 },
  { x:0, z:2100, altitude:240 }
];
const MISSION_PROFILES = {
  "mountain-city":{map:"mountain-city",title:"산악 도시 순환",difficulty:"보통",time:"약 3분",speed:[105,145],description:"산맥 아래의 굽은 항로를 따라 상승한 뒤 도시 활주로로 복귀합니다.",checkpoints:[{x:0,z:500,altitude:180},{x:85,z:900,altitude:300},{x:-75,z:1300,altitude:460},{x:105,z:1700,altitude:370},{x:0,z:2100,altitude:250}]},
  "ocean-islands":{map:"ocean-islands",title:"군도 해상 순찰",difficulty:"쉬움",time:"약 3분",speed:[100,140],description:"섬 사이를 넓게 선회하며 해상 항로를 확인하고 공항 섬으로 돌아옵니다.",checkpoints:[{x:0,z:500,altitude:160},{x:-120,z:900,altitude:220},{x:135,z:1300,altitude:285},{x:-90,z:1700,altitude:235},{x:0,z:2100,altitude:180}]},
  "desert-base":{map:"desert-base",title:"사막 저고도 침투",difficulty:"어려움",time:"약 2분 30초",speed:[115,155],description:"낮은 목표 고도를 유지하며 기지 외곽을 통과한 뒤 정밀 착륙합니다.",checkpoints:[{x:0,z:500,altitude:130},{x:95,z:900,altitude:175},{x:-110,z:1300,altitude:220},{x:120,z:1700,altitude:165},{x:0,z:2100,altitude:130}]},
  "night-city":{map:"night-city",title:"야간 항법 비행",difficulty:"어려움",time:"약 3분",speed:[100,140],description:"도시 항법등과 HUD를 따라 제한된 시야에서 야간 접근을 완료합니다.",checkpoints:[{x:0,z:500,altitude:200},{x:-75,z:900,altitude:270},{x:65,z:1300,altitude:330},{x:-45,z:1700,altitude:270},{x:0,z:2100,altitude:200}]}
};
const CHECKPOINT_RADIUS = 155; // 초보 조종을 고려한 수평 통과 판정 반지름(m)
const CHECKPOINT_VISUAL_RADIUS = 75; // 보이는 링의 물리 반지름(m). 판정 범위보다 작아 조준하기 쉽습니다.
const CHECKPOINT_ALTITUDE_TOLERANCE = 180; // 손 조종 오차를 고려한 통과 허용 고도 차이(ft)
const WIND_STREAK_START_SPEED = 70; // 이 속도부터 공기 흐름이 화면에 보이기 시작합니다.
const STALL_ENTER_SPEED = 58; // 공중에서 이 속도 아래면 실속 경고를 켭니다.
const STALL_EXIT_SPEED = 68; // 속도가 충분히 회복된 뒤 경고를 꺼 깜빡임을 막습니다.
const LANDING_VOICE_COOLDOWN = 1800; // 음성 경고가 서로 겹치지 않는 최소 간격(ms)
const CITY_DRAW_DISTANCE = 5600; // 도시 건물을 그리는 전방 거리(m)
const CITY_ROW_SPACING = 260; // 절차적으로 반복되는 도시 블록 간격(m)
const MAP_THEMES = {
  "mountain-city": {label:"산악 도시",sky:["#112c43","#456b80","#bdcdbf"],ground:["#718774","#425f55","#1c3734"],mountain:["#294b586e","#526f6f99"],grid:"166,190,147",road:"#263c3d99",crossRoad:"#2b414199",buildings:["#405653","#30494b","#253f46"],roof:"#65746a",side:"#172f36aa",window:"#d9e7ad",cloud:"222,233,226"},
  "ocean-islands": {label:"바다와 섬",sky:["#174264","#5792aa","#c2d7ce"],ground:["#276c84","#164f68","#082e48"],mountain:null,grid:"120,207,211",cloud:"229,240,236"},
  "desert-base": {label:"사막 기지",sky:["#244e67","#7798a0","#e6c692"],ground:["#ad895d","#8a613f","#513a2b"],mountain:["#7a4e3d88","#a36b4899"],grid:"226,190,126",road:"#59483caa",crossRoad:"#665044aa",buildings:["#806955","#6d5b4b","#584b40"],roof:"#aa9270",side:"#493e35bb",window:"#f1c66f",cloud:"239,221,188",desert:true},
  "night-city": {label:"야간 도시",sky:["#020712","#102337","#294252"],ground:["#13252b","#0b1d22","#040d12"],mountain:["#0b172566","#172b3299"],grid:"81,142,133",road:"#142b32cc",crossRoad:"#18323acc",buildings:["#1a3440","#142a37","#0d222e"],roof:"#34515a",side:"#081722dd",window:"#d7ef8b",cloud:"111,139,151",night:true}
};
// 생성형 이미지 자산은 장식용 지면층으로만 사용합니다. 로딩 전이나 실패 시에는 기존 절차 지형이 그대로 표시됩니다.
const TERRAIN_TEXTURE_URLS = {
  "mountain-city":"assets/terrain/terrain-mountain.png",
  "ocean-islands":"assets/terrain/terrain-ocean.png",
  "desert-base":"assets/terrain/terrain-desert.png",
  "night-city":"assets/terrain/terrain-night.png"
};
const MAPLIBRE_VERSION="5.6.0";
const MAPLIBRE_URL=`https://unpkg.com/maplibre-gl@${MAPLIBRE_VERSION}/dist/maplibre-gl.js`;
const OPENFREEMAP_STYLE="https://tiles.openfreemap.org/styles/liberty";
// Canvas 테마를 OpenFreeMap에서 보여줄 실제 지역에 연결합니다.
const EARTH_LOCATIONS = {
  "mountain-city":{label:"인스브루크",lat:47.2602,lon:11.3439},
  "ocean-islands":{label:"제주 해안",lat:33.4982,lon:126.4912},
  "desert-base":{label:"라스베이거스 사막",lat:36.1548,lon:-115.073},
  "night-city":{label:"서울 도심",lat:37.5665,lon:126.978}
};
const WEATHER_PRESETS = {
  clear:{label:"맑음",clouds:1,haze:0,wind:.08},
  sunset:{label:"석양",clouds:1.25,haze:.08,wind:.12,sky:["#162844","#a64f50","#f2a561"],sunset:true},
  overcast:{label:"흐림",clouds:2.25,haze:.18,wind:.24,sky:["#263844","#5d7075","#929d98"],overcast:true},
  rain:{label:"비",clouds:2.6,haze:.24,wind:.48,sky:["#132738","#405863","#75847f"],rain:1,overcast:true},
  fog:{label:"안개",clouds:.7,haze:.62,wind:.06,sky:["#61777d","#9caaa7","#c5cbc5"],fog:true}
};
const KNOTS_TO_MPS = 0.514444;
const FEET_TO_METERS = 0.3048;
const RUNWAY_NEAR_CLIP = 3;
const CONNECTIONS = [[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],[13,17],[0,17],[17,18],[18,19],[19,20]];

const $ = (id) => document.getElementById(id);
const canvas = $("flight-canvas");
const ctx = canvas.getContext("2d");
const video = $("webcam");
const landmarkCanvas = $("landmark-canvas");
const landmarkCtx = landmarkCanvas.getContext("2d");
const ui = Object.fromEntries(["speed-value","altitude-value","throttle-value","pitch-value","roll-value","elevator-value","vertical-speed","heading-value","right-status","left-status","stick-status","pinch-status","right-dot","left-dot","hand-count","throttle-fill","throttle-handle","stick-hint"].map(id => [id, $(id)]));
const flight = { speed: 120, altitude: 2400, throttle: 55, pitch: 0, roll: 0, heading: 300, verticalSpeed: 0, distance: 0 };
const lesson = { mode: "free", phase: "airborne", paused: false, brake: false, x: 0, z: 80, tookOff: false, takeoffNotified: false, touchdown: null, result: null, callouts: {}, approachAssistNotified:false };
const mission = { checkpoint:0, elapsed:0, returning:false, gateEffect:null, gateStatus:"", retries:0, profileId:"mountain-city" };
const stick = { x: 0, y: 0, grabbed: false, anchor: null, depthAnchor: null, depthShape: null };
const sound = { context:null, master:null, engineGain:null, engineOscillator:null, engineHarmonic:null, engineFilter:null, engineNoise:null, engineNoiseGain:null, engineNoiseFilter:null, engineTurbine:null, engineTurbineGain:null, engineAir:null, engineAirGain:null, engineAirFilter:null, engineCompressor:null, muted:false };
const effects = { stall:false, lastStallTone:0 };
const landingAlerts = { sink:false, lastSpoken:0 };
function newHand() { return { detected: false, point: null, pinch: false, lastSeen: 0, palmScale: null, palmShape: null, depthValid: false }; }
const hands = { Right: newHand(), Left: newHand() };
let width = 1, height = 1, dpr = 1;
let landmarker = null, stream = null, cameraActive = false;
let lastVideoTime = -1, lastInference = 0, lastFrame = 0, lastHud = 0;
let inferenceFailures = 0, messageTimer = 0;
let connectionStage = "idle";
let worldMap = (()=>{try{const saved=localStorage.getItem("aeronaut-map");return MAP_THEMES[saved]?saved:"mountain-city";}catch{return "mountain-city";}})();
let weatherMode = (()=>{try{const saved=localStorage.getItem("aeronaut-weather");return WEATHER_PRESETS[saved]?saved:"clear";}catch{return "clear";}})();
let selectedMissionId = (()=>{try{const saved=localStorage.getItem("aeronaut-mission");return MISSION_PROFILES[saved]?saved:"mountain-city";}catch{return "mountain-city";}})();
const realEarth={
  enabled:false,viewer:null,loading:false,libraryPromise:null,
  restore:(()=>{try{return localStorage.getItem("aeronaut-real-earth")==="true";}catch{return false;}})()
};
// 이전 Google 버전에서 브라우저에 저장했을 수 있는 키도 업그레이드 즉시 삭제합니다.
try{localStorage.removeItem("aeronaut-google-map-key");}catch{}
const terrainTextures={};
function terrainTexture(map) {
  if(terrainTextures[map])return terrainTextures[map];
  const image=new Image();image.decoding="async";image.src=TERRAIN_TEXTURE_URLS[map];
  terrainTextures[map]=image;return image;
}
// 기준 위치는 현재 카메라 연결 동안만 유지합니다. 이미지나 랜드마크는 저장하지 않습니다.
const calibration = { active: false, neutral: null, palmScale: null, palmShape: null, started: 0, held: 0, lastSample: 0, reference: null, referenceScale: null, sumScale: 0, sumShape: 0, sumX: 0, sumY: 0, count: 0, note: "이지 조종은 손을 보여주면 자동으로 시작합니다. C 키는 표시 중심을 맞출 때만 사용합니다." };

const clamp = (v, min, max) => Math.min(max, Math.max(min, v));
const lerp = (a, b, t) => a + (b - a) * t;
const damp = (a, b, speed, dt) => lerp(a, b, 1 - Math.exp(-speed * dt));
const radians = (degrees) => degrees * Math.PI / 180;
const degrees = (radiansValue) => radiansValue * 180 / Math.PI;
const signed = (v, digits = 1) => `${v >= 0 ? "+" : ""}${v.toFixed(digits)}`;
const angleDifference = (a, b) => ((a - b + 540) % 360) - 180;

function setEarthStatus(message,type="") {
  const status=$("earth-status");status.textContent=message;status.className=`earth-status${type?` ${type}`:""}`;
}

// MapLibre는 무료 지도를 켤 때만 내려받아 기존 Canvas 모드의 초기 로딩을 가볍게 유지합니다.
function loadMapLibrary() {
  if(window.maplibregl)return Promise.resolve(window.maplibregl);
  if(realEarth.libraryPromise)return realEarth.libraryPromise;
  realEarth.libraryPromise=new Promise((resolve,reject)=>{
    const script=document.createElement("script");
    const timeout=setTimeout(()=>reject(new Error("MapLibre 로딩 시간이 초과되었습니다.")),30000);
    script.src=MAPLIBRE_URL;script.async=true;
    script.onload=()=>{clearTimeout(timeout);window.maplibregl?resolve(window.maplibregl):reject(new Error("MapLibre를 시작할 수 없습니다."));};
    script.onerror=()=>{clearTimeout(timeout);reject(new Error("MapLibre CDN을 불러오지 못했습니다."));};
    document.head.appendChild(script);
  }).catch(error=>{realEarth.libraryPromise=null;throw error;});
  return realEarth.libraryPromise;
}

function destroyEarthViewer() {
  if(realEarth.viewer)realEarth.viewer.remove();
  realEarth.viewer=null;
  $("map-container").replaceChildren();
}

function addFreeMapBuildings(map) {
  if(map.getLayer("aeronaut-3d-buildings"))return;
  const layers=map.getStyle()?.layers||[];
  const buildingLayer=layers.find(layer=>layer["source-layer"]==="building"&&layer.source);
  if(!buildingLayer)return;
  const firstLabel=layers.find(layer=>layer.type==="symbol"&&layer.layout?.["text-field"]);
  map.addLayer({
    id:"aeronaut-3d-buildings",type:"fill-extrusion",source:buildingLayer.source,"source-layer":"building",minzoom:13,
    paint:{
      "fill-extrusion-color":["interpolate",["linear"],["zoom"],13,"#6f8581",17,"#aec1ba"],
      "fill-extrusion-height":["coalesce",["to-number",["get","render_height"]],["to-number",["get","height"]],10],
      "fill-extrusion-base":["coalesce",["to-number",["get","render_min_height"]],["to-number",["get","min_height"]],0],
      "fill-extrusion-opacity":.78
    }
  },firstLabel?.id);
}

async function createEarthViewer() {
  const maplibregl=await loadMapLibrary();
  if(realEarth.viewer)return realEarth.viewer;
  destroyEarthViewer();
  const location=EARTH_LOCATIONS[worldMap];
  const viewer=new maplibregl.Map({
    container:"map-container",style:OPENFREEMAP_STYLE,center:[location.lon,location.lat],zoom:15.5,
    bearing:flight.heading,pitch:78,roll:-flight.roll,maxPitch:85,rollEnabled:true,
    interactive:false,attributionControl:true,renderWorldCopies:false,
    localIdeographFontFamily:"Noto Sans, Malgun Gothic, sans-serif"
  });
  // 스타일에 아이콘이 빠진 소규모 POI는 투명 픽셀로 대체해 렌더링 경고와 불필요한 재시도를 막습니다.
  viewer.on("styleimagemissing",event=>{
    if(!viewer.hasImage(event.id))viewer.addImage(event.id,{width:1,height:1,data:new Uint8Array([0,0,0,0])});
  });
  realEarth.viewer=viewer;
  await new Promise((resolve,reject)=>{
    const timeout=setTimeout(()=>reject(new Error("무료 지도 응답 시간이 초과되었습니다.")),30000);
    viewer.once("load",()=>{clearTimeout(timeout);try{addFreeMapBuildings(viewer);}catch(error){console.warn("3D 건물 레이어 생략",error);}resolve();});
    viewer.once("error",event=>{if(!viewer.loaded()){clearTimeout(timeout);reject(event.error||new Error("무료 지도 데이터를 불러오지 못했습니다."));}});
  });
  return viewer;
}

async function enableRealEarth() {
  if(realEarth.loading)return;
  realEarth.loading=true;$("earth-enable-button").disabled=true;setEarthStatus("무료 실제 지도를 불러오는 중입니다…");
  // 숨겨진 요소는 WebGL 크기를 계산할 수 없으므로 초기화 중에도 컨테이너를 열어 둡니다.
  $("map-container").hidden=false;
  try {
    await createEarthViewer();
    realEarth.enabled=true;realEarth.restore=true;
    try{localStorage.removeItem("aeronaut-google-map-key");localStorage.setItem("aeronaut-real-earth","true");}catch{}
    $("map-container").hidden=false;$("flight").classList.add("real-earth");
    $("earth-button").textContent="무료 지도 ON";$("earth-button").setAttribute("aria-pressed","true");
    $("render-mode-label").textContent="FREE MAP SIMULATOR";
    setEarthStatus(`연결됨 · ${EARTH_LOCATIONS[worldMap].label} 무료 3D 지도`,"active");
    realEarth.viewer.resize();syncRealEarthCamera(true);setEarthPanel(false,false);
    showMessage(`무료 실제 지도 연결 · ${EARTH_LOCATIONS[worldMap].label}`,false,4500);
  } catch(error) {
    console.error("무료 지도 초기화 실패",error);
    realEarth.enabled=false;destroyEarthViewer();$("map-container").hidden=true;$("flight").classList.remove("real-earth");
    $("earth-button").textContent="무료 지도 오류";$("earth-button").setAttribute("aria-pressed","false");
    setEarthStatus(`연결 실패 · ${error?.message||"인터넷 연결을 확인해주세요."}`,"error");
  } finally {realEarth.loading=false;$("earth-enable-button").disabled=false;}
}

function disableRealEarth(notify=true) {
  realEarth.enabled=false;realEarth.restore=false;
  try{localStorage.setItem("aeronaut-real-earth","false");}catch{}
  $("map-container").hidden=true;$("flight").classList.remove("real-earth");
  $("earth-button").textContent="무료 지도 OFF";$("earth-button").setAttribute("aria-pressed","false");
  $("render-mode-label").textContent="CANVAS SIMULATOR";
  setEarthStatus("Canvas 배경을 사용 중입니다. 무료 지도를 언제든 다시 켤 수 있습니다.");
  if(notify){setEarthPanel(false,false);showMessage("Canvas 배경으로 돌아왔습니다.",false,3500);}
}

function syncRealEarthCamera(force=false) {
  if(!realEarth.enabled||!realEarth.viewer)return;
  const location=EARTH_LOCATIONS[worldMap];if(!location)return;
  const runwayHeading=radians(RUNWAY.heading),east=Math.sin(runwayHeading)*lesson.z+Math.cos(runwayHeading)*lesson.x;
  const north=Math.cos(runwayHeading)*lesson.z-Math.sin(runwayHeading)*lesson.x;
  const latitude=location.lat+north/110540;
  const longitude=location.lon+east/(111320*Math.cos(radians(location.lat)));
  const zoom=clamp(18.7-Math.log2(Math.max(35,flight.altitude+35)/70),12.2,18);
  realEarth.viewer.jumpTo({center:[longitude,latitude],bearing:flight.heading,pitch:clamp(78+flight.pitch*.18,70,84),roll:-flight.roll,zoom});
  if(force)realEarth.viewer.triggerRepaint();
}

// 외부 음원 파일 없이 Web Audio 노드만 사용합니다. 브라우저 정책상 첫 사용자 입력 뒤에 시작됩니다.
function ensureAudio() {
  if(sound.context) {
    if(sound.context.state==="suspended") sound.context.resume().catch(()=>{});
    return sound.context;
  }
  const AudioContextClass=window.AudioContext||window.webkitAudioContext;
  if(!AudioContextClass) return null;
  try {
    const context=new AudioContextClass();
    const master=context.createGain();master.gain.value=sound.muted?0:.28;master.connect(context.destination);
    const filter=context.createBiquadFilter();filter.type="lowpass";filter.frequency.value=260;filter.Q.value=.55;
    const engineGain=context.createGain();engineGain.gain.value=0;
    // 날카로운 톱니파 대신 저음 삼각파와 약한 배음을 사용합니다.
    const oscillator=context.createOscillator();oscillator.type="triangle";oscillator.frequency.value=48;
    const harmonic=context.createOscillator(), harmonicGain=context.createGain();
    harmonic.type="sine";harmonic.frequency.value=96;harmonicGain.gain.value=.16;
    oscillator.connect(filter);harmonic.connect(harmonicGain);harmonicGain.connect(filter);
    filter.connect(engineGain);oscillator.start();harmonic.start();

    // 저역 성분이 많은 연속 노이즈로 제트 흡기/바람 느낌을 더합니다.
    const noiseBuffer=context.createBuffer(1,context.sampleRate*2,context.sampleRate);
    const noiseData=noiseBuffer.getChannelData(0);let previousNoise=0;
    for(let i=0;i<noiseData.length;i++) {
      previousNoise=(previousNoise+(Math.random()*2-1)*.02)/1.02;
      noiseData[i]=previousNoise*3.2;
    }
    const noise=context.createBufferSource(), noiseFilter=context.createBiquadFilter(), noiseGain=context.createGain();
    noise.buffer=noiseBuffer;noise.loop=true;noiseFilter.type="bandpass";noiseFilter.frequency.value=420;noiseFilter.Q.value=.55;noiseGain.gain.value=.2;
    noise.connect(noiseFilter);noiseFilter.connect(noiseGain);noiseGain.connect(engineGain);noise.start();

    // 터빈 휘파람 성분은 작게 유지하고 출력에 따라 빠르게 회전하는 인상을 줍니다.
    const turbine=context.createOscillator(), turbineGain=context.createGain();
    turbine.type="sine";turbine.frequency.value=190;turbineGain.gain.value=.018;
    turbine.connect(turbineGain);turbineGain.connect(engineGain);turbine.start();

    // 고역 배기 노이즈는 80% 이상에서 애프터버너처럼 강하게 열립니다.
    const airBuffer=context.createBuffer(1,context.sampleRate*2,context.sampleRate);
    const airData=airBuffer.getChannelData(0);
    for(let i=0;i<airData.length;i++) airData[i]=Math.random()*2-1;
    const air=context.createBufferSource(), airFilter=context.createBiquadFilter(), airGain=context.createGain();
    air.buffer=airBuffer;air.loop=true;airFilter.type="bandpass";airFilter.frequency.value=900;airFilter.Q.value=.45;airGain.gain.value=.015;
    air.connect(airFilter);airFilter.connect(airGain);airGain.connect(engineGain);air.start();

    const compressor=context.createDynamicsCompressor();
    compressor.threshold.value=-24;compressor.knee.value=16;compressor.ratio.value=4;compressor.attack.value=.012;compressor.release.value=.22;
    engineGain.connect(compressor);compressor.connect(master);
    Object.assign(sound,{context,master,engineGain,engineOscillator:oscillator,engineHarmonic:harmonic,engineFilter:filter,engineNoise:noise,engineNoiseGain:noiseGain,engineNoiseFilter:noiseFilter,engineTurbine:turbine,engineTurbineGain:turbineGain,engineAir:air,engineAirGain:airGain,engineAirFilter:airFilter,engineCompressor:compressor});
    context.resume().catch(()=>{});
    return context;
  } catch(error) {
    console.warn("오디오 초기화 실패",error);
    return null;
  }
}

function updateEngineSound() {
  const context=sound.context;
  if(!context||!sound.engineGain||context.state==="closed") return;
  const throttle=clamp(flight.throttle/100,0,1);
  const afterburner=clamp((throttle-.78)/.22,0,1);
  const activeGain=lesson.paused||lesson.result ? .006 : .014+throttle*.036;
  sound.engineGain.gain.setTargetAtTime(sound.muted?0:activeGain,context.currentTime,.14);
  sound.engineOscillator.frequency.setTargetAtTime(42+throttle*35,context.currentTime,.16);
  sound.engineHarmonic.frequency.setTargetAtTime(84+throttle*70,context.currentTime,.16);
  sound.engineFilter.frequency.setTargetAtTime(180+throttle*270,context.currentTime,.18);
  sound.engineNoiseGain.gain.setTargetAtTime(.22+throttle*.58,context.currentTime,.2);
  sound.engineNoiseFilter.frequency.setTargetAtTime(280+throttle*720,context.currentTime,.2);
  sound.engineNoiseFilter.Q.setTargetAtTime(.55+throttle*.25,context.currentTime,.2);
  sound.engineTurbine.frequency.setTargetAtTime(190+throttle*610,context.currentTime,.24);
  sound.engineTurbineGain.gain.setTargetAtTime(.018+Math.pow(throttle,1.4)*.085,context.currentTime,.2);
  sound.engineAirFilter.frequency.setTargetAtTime(850+throttle*2200,context.currentTime,.18);
  sound.engineAirFilter.Q.setTargetAtTime(.45+afterburner*.28,context.currentTime,.16);
  sound.engineAirGain.gain.setTargetAtTime(.015+throttle*.16+afterburner*.28,context.currentTime,.18);
}

function playTone(frequency,duration=.15,volume=.12,delay=0,type="sine") {
  if(sound.muted) return;
  const context=ensureAudio();
  if(!context||!sound.master) return;
  const start=context.currentTime+delay;
  const oscillator=context.createOscillator(), gain=context.createGain();
  oscillator.type=type;oscillator.frequency.setValueAtTime(frequency,start);
  gain.gain.setValueAtTime(.0001,start);gain.gain.exponentialRampToValueAtTime(volume,start+.018);
  gain.gain.exponentialRampToValueAtTime(.0001,start+duration);
  oscillator.connect(gain);gain.connect(sound.master);oscillator.start(start);oscillator.stop(start+duration+.03);
}

// 운영체제의 음성 합성을 사용해 외부 음원 없이 항공기식 콜아웃을 냅니다.
function speakCallout(text) {
  if(sound.muted||!("speechSynthesis" in window)||typeof SpeechSynthesisUtterance==="undefined") return;
  try {
    const utterance=new SpeechSynthesisUtterance(text);
    utterance.lang="en-US";utterance.rate=.92;utterance.pitch=.72;utterance.volume=.9;
    const voices=window.speechSynthesis.getVoices();
    utterance.voice=voices.find(voice=>/david|guy|male/i.test(voice.name)&&voice.lang.startsWith("en"))
      ||voices.find(voice=>voice.lang.startsWith("en"))||null;
    window.speechSynthesis.speak(utterance);
  } catch(error) { console.warn("음성 콜아웃 재생 실패",error); }
}

function playGateSound(success,final=false) {
  if(success) {
    [0,1,2].forEach((step,index)=>playTone((final?520:440)*Math.pow(1.25,step),.18,.13,index*.11,"triangle"));
  } else {
    playTone(220,.24,.14,0,"square");playTone(145,.32,.12,.18,"square");
  }
}

function setSoundMuted(muted) {
  sound.muted=muted;
  const context=ensureAudio();
  if(context&&sound.master) sound.master.gain.setTargetAtTime(muted?0:.28,context.currentTime,.03);
  const button=$("audio-button");button.textContent=muted?"사운드 OFF":"사운드 ON";button.setAttribute("aria-pressed",String(!muted));
  if(muted&&"speechSynthesis" in window) window.speechSynthesis.cancel();
}

function startLesson(mode) {
  clearHands(); stick.x = 0; stick.y = 0;
  const groundStart = mode === "takeoff" || mode === "mission";
  Object.assign(lesson, { mode, phase: groundStart ? "ground" : "airborne", paused: mode !== "free", brake: false, x: 0, z: mode === "landing" ? -2400 : 80, tookOff: false, takeoffNotified: false, touchdown: null, result: null, callouts: {}, approachAssistNotified:false });
  Object.assign(mission,{checkpoint:0,elapsed:0,returning:false,gateEffect:null,gateStatus:"",retries:0});
  const preset = groundStart ? { speed: 0, altitude: 0, throttle: 0, pitch: 0 }
    : mode === "landing" ? { speed: 100, altitude: 460, throttle: 38, pitch: -3 }
      : { speed: 120, altitude: 2400, throttle: 55, pitch: 0 };
  Object.assign(flight, preset, { roll: 0, heading: RUNWAY.heading, verticalSpeed: 0, distance: 0 });
  effects.stall=false;effects.lastStallTone=0;$("flight-warning").hidden=true;
  Object.assign(landingAlerts,{sink:false,lastSpoken:0});
  if("speechSynthesis" in window) window.speechSynthesis.cancel();
  $("flight-result").hidden = true;
  $("flight").classList.toggle("training", mode !== "free");
  showMessage(mode === "takeoff" ? "지상 출발 준비. 손 중심과 출력을 맞춘 뒤 ‘비행 시작’을 누르세요."
    : mode === "mission" ? "미션 준비 · 이륙 후 GATE 1부터 순서대로 통과하고 활주로로 복귀하세요."
    : mode === "landing" ? "활주로 2.4km 앞에서 대기합니다. 손 조종을 준비하고 ‘비행 시작’을 누르세요."
      : "자유 비행으로 돌아왔습니다.", false, 6000);
  updateHud();
}

function finishLesson(success, description) {
  if (lesson.result) return;
  const touchdown = lesson.touchdown;
  const landingScore = success ? clamp(100-touchdown.sink*3-touchdown.bank*1.5-touchdown.headingError-Math.abs(touchdown.x)*.5,0,100) : 0;
  const timeScore = clamp(100-Math.max(0,mission.elapsed-120)*.35,30,100);
  const score = success ? Math.round(lesson.mode === "mission" ? landingScore*.75+timeScore*.25 : landingScore) : 0;
  if(success&&lesson.mode==="mission") saveMissionBest(mission.profileId,score);
  lesson.result = { success, score, description };
  lesson.phase = success ? "complete" : "failed";
  releaseStick();
  $("flight-result").hidden = false;
  $("flight-result").classList.toggle("failed", !success);
  $("result-title").textContent = success ? `${lesson.mode === "mission" ? "미션 완료" : "착륙 완료"} · ${score}점` : lesson.mode === "mission" ? "미션 종료" : "훈련 종료";
  $("result-description").textContent = description;
  $("result-metrics").textContent = touchdown
    ? `${lesson.mode === "mission" ? `미션  ${MISSION_PROFILES[mission.profileId]?.title||"통합 미션"}\n체크포인트  ${mission.checkpoint} / ${MISSION_CHECKPOINTS.length}\n비행 시간  ${formatMissionTime(mission.elapsed)}\n` : ""}접지 속도  ${touchdown.speed.toFixed(0)} KTS\n접지 하강률  ${(touchdown.sink * 60).toFixed(0)} FT/MIN\n접지 Pitch  ${touchdown.pitch.toFixed(1)}°\n접지 기울기  ${touchdown.bank.toFixed(1)}°\n중심선 편차  ${Math.abs(touchdown.x).toFixed(1)} M`
    : `속도  ${flight.speed.toFixed(0)} KTS\n활주로 진행  ${lesson.z.toFixed(0)} M`;
  clearTimeout(messageTimer); $("message").hidden = true;
  // 결과창이 뜨는 프레임에도 고도와 속도를 즉시 갱신해 접지 전 값이 남지 않게 합니다.
  updateHud();
  $("flight-result").focus({ preventScroll: true });
}

function onRunway(x, z) { return Math.abs(x) <= RUNWAY.width / 2 - 3 && z >= 0 && z <= RUNWAY.length; }

function assessTouchdown(x, z, sink) {
  const headingError = Math.min(Math.abs(angleDifference(flight.heading, RUNWAY.heading)), Math.abs(angleDifference(flight.heading, (RUNWAY.heading + 180) % 360)));
  lesson.touchdown = { x, z, sink, speed: flight.speed, pitch: flight.pitch, bank: Math.abs(flight.roll), headingError };
  if (lesson.mode === "mission" && !mission.returning) return `체크포인트 ${mission.checkpoint+1}을 통과하기 전에 착륙했습니다.`;
  if (!onRunway(x, z)) return "활주로 밖에 접지했습니다. 중심선에 정렬하고 활주로 위에서 내려오세요.";
  if (sink > RUNWAY.maxSink) return easyControlEnabled()
    ? "하강 속도가 너무 빨랐습니다. 접지 직전 오른손을 조금 올려 하강률을 줄이세요."
    : "하강 속도가 너무 빨랐습니다. 접지 직전 조종간을 조금 당겨 하강률을 줄이세요.";
  if (Math.abs(flight.roll) > RUNWAY.maxBank) return "기울기가 큰 상태로 접지했습니다. 양쪽 날개를 수평으로 맞추세요.";
  if (headingError > RUNWAY.maxHeadingError) return "활주로 방향과 크게 어긋났습니다. 방위를 300° 또는 120°에 맞추세요.";
  if (flight.speed > RUNWAY.maxLandingSpeed) return "접지 속도가 너무 높았습니다. 접근 중 스로틀을 줄여 80~100 KTS를 목표로 하세요.";
  if (flight.speed < RUNWAY.minLandingSpeed) return "접지 전 속도가 너무 낮아졌습니다. 접근 중 적당한 출력을 유지하세요.";
  if (lesson.touchdown.pitch < -3 || lesson.touchdown.pitch > 12) return "접지 자세가 과도했습니다. 지면 가까이에서는 기수를 완만하게 들어주세요.";
  return null;
}

function getLandingGuidance() {
  const forward = Math.cos(radians(flight.heading-RUNWAY.heading)) >= 0;
  const targetZ = forward ? 300 : RUNWAY.length-300;
  const dx = -lesson.x, dz = targetZ-lesson.z;
  const distance = Math.max(0,Math.hypot(dx,dz));
  const desiredHeading = (RUNWAY.heading+degrees(Math.atan2(dx,dz))+360)%360;
  const desiredAltitude = distance*Math.tan(radians(3))/FEET_TO_METERS;
  const headingError = angleDifference(desiredHeading,flight.heading);
  const altitudeError = flight.altitude-desiredAltitude;
  const flare = flight.altitude <= 60 && distance < 750;
  const stage = flight.altitude <= 12 ? "접지 직전"
    : flare ? "플레어"
      : distance < 900 ? "최종 접근"
        : distance < 1800 ? "안정 접근" : "진입 접근";
  const targetSpeed = flare ? 82 : distance < 1000 ? 88 : 95;
  return { forward,targetZ,distance,desiredHeading,desiredAltitude,headingError,altitudeError,flare,stage,targetSpeed };
}

function landingModeActive() {
  return lesson.mode === "landing" || (lesson.mode === "mission" && mission.returning);
}

function papiGuidance() {
  if(!landingModeActive()||lesson.phase!=="airborne") return null;
  const guidance=getLandingGuidance(),error=guidance.altitudeError;
  const whiteCount=error>100?4:error>35?3:error>=-35?2:error>=-100?1:0;
  return {...guidance,whiteCount,label:whiteCount===2?"ON GLIDE":whiteCount>2?"HIGH":"LOW"};
}

// 현재 자세를 그대로 유지한다고 가정한 단순 예상 접지점입니다.
function predictTouchdown() {
  if(!landingModeActive()||lesson.phase!=="airborne"||flight.verticalSpeed>=-.5||flight.altitude<=0) return null;
  const seconds=clamp(flight.altitude/-flight.verticalSpeed,0,60);
  const travel=flight.speed*KNOTS_TO_MPS*seconds;
  const direction=radians(flight.heading-RUNWAY.heading);
  const x=lesson.x+Math.sin(direction)*travel;
  const z=lesson.z+Math.cos(direction)*travel;
  const guidance=getLandingGuidance();
  const alongError=(z-guidance.targetZ)*(guidance.forward?1:-1);
  const laterallyAligned=Math.abs(x)<=RUNWAY.width/2-3;
  const status=!laterallyAligned?"OFF RUNWAY":alongError < -250?"SHORT":alongError > 450?"LONG":"TOUCHDOWN";
  return {x,z,seconds,status,safe:status==="TOUCHDOWN"};
}

function updateLandingCallouts() {
  if (!landingModeActive() || lesson.phase !== "airborne") return;
  const now=performance.now(),thresholds=[100,50,30,20,10];
  const crossed=thresholds.filter(altitude=>flight.altitude<=altitude&&!lesson.callouts[altitude]);
  if(crossed.length) {
    crossed.forEach(altitude=>lesson.callouts[altitude]=true);
    const altitude=Math.min(...crossed);
    const spoken={100:"one hundred",50:"fifty",30:"thirty",20:"twenty",10:"ten"}[altitude];
    const text = altitude === 50
      ? (easyControlEnabled() ? "50 FT · FLARE · 오른손을 조금 올려 기수를 완만하게 드세요." : "50 FT · FLARE · 조종간을 조금 당겨 기수를 완만하게 드세요.")
      : `${altitude} FT`;
    playTone(altitude===50?720:560,.12,altitude===50?.14:.09,0,"triangle");
    speakCallout(spoken);landingAlerts.lastSpoken=now;
    showMessage(text,false,altitude===50?5000:1800);
    return;
  }

  const sinkDanger=flight.altitude<300&&-flight.verticalSpeed*60>RUNWAY.maxSink*60;
  landingAlerts.sink=sinkDanger;
  if(now-landingAlerts.lastSpoken<LANDING_VOICE_COOLDOWN) return;
  if(sinkDanger) {
    landingAlerts.lastSpoken=now;
    playTone(240,.16,.12,0,"square");playTone(180,.18,.11,.18,"square");speakCallout("sink rate");
  }
}

function formatMissionTime(seconds) {
  const total=Math.max(0,Math.floor(seconds));
  return `${String(Math.floor(total/60)).padStart(2,"0")}:${String(total%60).padStart(2,"0")}`;
}

function updateMission(previous) {
  if (lesson.mode !== "mission") return;
  if (lesson.phase !== "airborne" || mission.returning) return;
  const checkpoint=MISSION_CHECKPOINTS[mission.checkpoint];
  if (!checkpoint) { mission.returning=true; return; }
  const horizontal=Math.hypot(checkpoint.x-lesson.x,checkpoint.z-lesson.z);
  const altitudeError=Math.abs(flight.altitude-checkpoint.altitude);
  mission.gateStatus = horizontal<=CHECKPOINT_RADIUS*1.8 && altitudeError>CHECKPOINT_ALTITUDE_TOLERANCE
    ? `링 근처 · ${flight.altitude<checkpoint.altitude?"고도를 올리세요":"고도를 내리세요"} (${Math.round(altitudeError)} FT 차이)`
    : "";
  // 목표 주변 구체에 닿았을 때가 아니라 링이 놓인 z 평면을 가로지른 순간을 판정합니다.
  // 그래서 링이 화면 밖으로 커진 다음 기체 뒤로 빠지는 실제 통과 움직임이 생깁니다.
  const zTravel=lesson.z-previous.z;
  const crossingTime=Math.abs(zTravel)>0.0001 ? (checkpoint.z-previous.z)/zTravel : -1;
  const crossedPlane=crossingTime>=0 && crossingTime<=1;
  const crossingX=crossedPlane ? lerp(previous.x,lesson.x,crossingTime) : lesson.x;
  const crossingAltitude=crossedPlane ? lerp(previous.altitude,flight.altitude,crossingTime) : flight.altitude;
  const lateralError=Math.abs(crossingX-checkpoint.x);
  const crossingAltitudeError=Math.abs(crossingAltitude-checkpoint.altitude);
  if(!crossedPlane) return;

  if(lateralError>CHECKPOINT_RADIUS||crossingAltitudeError>CHECKPOINT_ALTITUDE_TOLERANCE) {
    const missedGate=mission.checkpoint+1;
    const retryPoint=mission.checkpoint===0
      ? {x:0,z:250,altitude:100}
      : MISSION_CHECKPOINTS[mission.checkpoint-1];
    const reason=lateralError>CHECKPOINT_RADIUS
      ? `좌우 ${Math.round(lateralError)} M 이탈`
      : `고도 ${Math.round(crossingAltitudeError)} FT 차이`;
    Object.assign(lesson,{x:retryPoint.x,z:retryPoint.z});
    Object.assign(flight,{speed:90,altitude:retryPoint.altitude,throttle:Math.min(flight.throttle,50),pitch:0,roll:0,heading:RUNWAY.heading,verticalSpeed:0});
    stick.x=0;stick.y=0;
    mission.retries++;
    mission.gateStatus=`GATE ${missedGate} 재시도 · ${reason}`;
    mission.gateEffect={gate:missedGate,started:performance.now(),final:false,success:false};
    playGateSound(false);
    showMessage(`GATE ${missedGate} 통과 실패 (${reason}) · 이전 게이트 위치로 돌아왔습니다.`,true,5000);
    return;
  }

  const clearedGate=mission.checkpoint+1;
  mission.checkpoint++;
  mission.gateStatus="";
  mission.gateEffect={gate:clearedGate,started:performance.now(),final:mission.checkpoint>=MISSION_CHECKPOINTS.length,success:true};
  playGateSound(true,mission.checkpoint>=MISSION_CHECKPOINTS.length);
  if (mission.checkpoint>=MISSION_CHECKPOINTS.length) {
    mission.returning=true;
    // 링 코스를 마친 뒤 먼 거리를 되돌아오지 않도록 짧은 최종 접근 위치로 전환합니다.
    Object.assign(lesson,{x:0,z:-1600,callouts:{}});
    Object.assign(flight,{speed:95,altitude:310,throttle:35,pitch:-3,roll:0,heading:RUNWAY.heading,verticalSpeed:0});
    stick.x=0;stick.y=0;
    showMessage("모든 체크포인트 통과 · 가까운 착륙 접근 위치로 전환했습니다.",false,7000);
  } else {
    const next=MISSION_CHECKPOINTS[mission.checkpoint];
    const nextHeading=(RUNWAY.heading+degrees(Math.atan2(next.x-lesson.x,next.z-lesson.z))+360)%360;
    const nextBearing=angleDifference(nextHeading,flight.heading);
    const turn=Math.abs(nextBearing)<8 ? "정면 유지" : `${nextBearing>0?"오른쪽":"왼쪽"} ${Math.abs(Math.round(nextBearing))}° 선회`;
    showMessage(`GATE ${clearedGate} CLEAR · 다음 GATE ${mission.checkpoint+1}: ${turn}`,false,5000);
  }
}

// 활주로 평면 좌표는 m, HUD 고도는 ft, 속도는 KTS입니다. 매 프레임 단위를 변환합니다.
function updateRunwayFlight(dt) {
  if (lesson.mode === "mission") mission.elapsed+=dt;
  const response=controlResponse();
  const applied=appliedStickInput();
  const grounded = lesson.phase === "ground" || lesson.phase === "rollout";
  flight.roll = damp(flight.roll, grounded ? 0 : applied.x * MAX_ROLL, response.rollSpeed, dt);
  // 지상에서는 속도가 나기 전까지 기수가 들리지 않습니다. 이륙 속도에서 당겨야 회전합니다.
  let targetPitch = grounded ? 0 : applied.y * MAX_PITCH;
  // 이륙 속도 이후에는 작은 상승 입력도 회전 의도로 보고 최소 5°를 목표로 합니다.
  // 손 입력이 약해 목표 각도가 1~2°에 머무르던 문제를 줄입니다.
  if (lesson.phase === "ground" && flight.speed >= RUNWAY.rotationSpeed && applied.y > 0.03) {
    targetPitch = Math.max(targetPitch, 6 + applied.y * 8);
  }
  // 지면을 떠난 직후 보조가 사라져 Pitch가 다시 1~2°로 내려가던 현상을 막습니다.
  // 고도 100 FT까지 상승 입력을 유지하면 안정적인 초기 상승 자세를 제공합니다.
  if (lesson.phase === "airborne" && lesson.tookOff && !lesson.takeoffNotified && applied.y > 0.08) {
    targetPitch = Math.max(targetPitch, 6);
  }
  flight.pitch = damp(flight.pitch, targetPitch, response.pitchSpeed, dt);
  if (grounded) {
    const braking = lesson.brake || flight.throttle < 10;
    const acceleration = flight.throttle * 0.10 - 1.8 - flight.speed * 0.012 - (braking ? 12 : 0);
    flight.speed = clamp(flight.speed + acceleration * dt, 0, 220);
    flight.heading = (flight.heading + applied.x * 10 * clamp(flight.speed / 35, 0, 1) * dt + 360) % 360;
    flight.altitude = 0; flight.verticalSpeed = 0;
    if (lesson.phase === "ground" && !lesson.brake && flight.speed >= RUNWAY.rotationSpeed && flight.pitch >= RUNWAY.rotationPitch) {
      lesson.phase = "airborne"; lesson.tookOff = true;
    }
  } else {
    flight.speed = damp(flight.speed, clamp(30 + flight.throttle * 1.8 - flight.pitch * 0.3, 25, 220), 0.24, dt);
    flight.heading = (flight.heading + Math.sin(radians(flight.roll)) * flight.speed * 0.06 * dt + 360) % 360;
  }
  const previous = { x: lesson.x, z: lesson.z, altitude: flight.altitude };
  const direction = radians(flight.heading - RUNWAY.heading);
  const travel = flight.speed * KNOTS_TO_MPS * dt;
  lesson.x += Math.sin(direction) * travel;
  lesson.z += Math.cos(direction) * travel;
  if (lesson.phase === "airborne") {
    const sinkFromLowSpeed = Math.max(0, 65 - flight.speed) * 0.45;
    flight.verticalSpeed = Math.sin(radians(flight.pitch)) * flight.speed * KNOTS_TO_MPS / FEET_TO_METERS - sinkFromLowSpeed;
    const landingActive = lesson.mode === "landing" || (lesson.mode === "mission" && mission.returning);
    // 최종 접근에서 기수를 수평에 가깝게만 만들어도 지면 효과가 하강률을 완화합니다.
    // Pitch -2° 이하의 급강하는 보조하지 않아 사용자가 상승 입력으로 플레어해야 합니다.
    if (landingActive && flight.altitude < 120 && flight.pitch > -2 && flight.verticalSpeed < 0) {
      const proximity=clamp(1-flight.altitude/120,0,1);
      const flareInput=clamp((flight.pitch+2)/6,0,1);
      const assist=clamp(proximity*(.6+flareInput*.4),0,1);
      const maximumSink=lerp(RUNWAY.maxSink,4,assist);
      flight.verticalSpeed=Math.max(flight.verticalSpeed,-maximumSink);
    }
    flight.altitude += flight.verticalSpeed * dt;
    // 최종 접근에서 활주로 문턱에 닿기 전에 지면으로 내려가면 즉시 실패하던 문제를 막습니다.
    // 활주로 바깥의 접근 구간에서는 거리에 비례한 낮은 안전 고도를 유지해 플레어할 시간을 줍니다.
    const outsideRunwayEnds=lesson.z<0||lesson.z>RUNWAY.length;
    if(landingActive&&outsideRunwayEnds) {
      const distanceToThreshold=lesson.z<0?-lesson.z:lesson.z-RUNWAY.length;
      const approachFloor=clamp(distanceToThreshold*.08,6,100);
      if(flight.altitude<approachFloor) {
        flight.altitude=approachFloor;
        flight.verticalSpeed=Math.max(flight.verticalSpeed,-1.5);
        if(!lesson.approachAssistNotified) {
          lesson.approachAssistNotified=true;
          showMessage("활주로 문턱 전 저고도 보조 · 기수를 조금 올려 진입을 계속하세요.",false,5000);
        }
      }
    }
    // 이동과 고도 변화가 끝난 최신 위치로 링 통과를 판정합니다.
    updateMission(previous);
    updateLandingCallouts();
    if (flight.altitude <= 0 && flight.verticalSpeed <= 0) {
      // 지면을 통과한 프레임은 접지 시점을 보간하여 활주로 끝 판정 오차를 줄입니다.
      const fraction = previous.altitude > 0 ? clamp(previous.altitude / (previous.altitude - flight.altitude), 0, 1) : 0;
      lesson.x = lerp(previous.x, lesson.x, fraction); lesson.z = lerp(previous.z, lesson.z, fraction);
      const reason = assessTouchdown(lesson.x, lesson.z, Math.max(0, -flight.verticalSpeed));
      flight.altitude = 0; flight.verticalSpeed = 0;
      if (reason) { finishLesson(false, reason); return; }
      lesson.phase = "rollout";
      lesson.brake=true;
      flight.throttle=Math.min(flight.throttle,5);
      playTone(110,.35,.18,0,"sine");playTone(440,.25,.1,.22,"triangle");
      showMessage("접지 성공! 스로틀을 줄이고 브레이크를 자동으로 작동합니다.", false, 7000);
    }
    if (lesson.tookOff && !lesson.takeoffNotified && flight.altitude >= 100) {
      lesson.takeoffNotified = true;
      playTone(440,.18,.11,0,"triangle");playTone(660,.28,.12,.14,"triangle");
      showMessage(lesson.mode === "mission"
        ? "이륙 성공 · GATE 1 유도 마름모를 따라가세요."
        : "이륙 성공 · 고도 100 FT 통과! 선회해 돌아오거나 ‘착륙 연습’으로 접근을 시작하세요.", false, 8000);
    }
  } else if (!onRunway(lesson.x, lesson.z)) {
    finishLesson(false, lesson.phase === "rollout" ? "정지하기 전에 활주로를 벗어났습니다. 접지 후 출력을 내리고 제동하세요." : "이륙 전에 활주로를 벗어났습니다. 중심선을 유지하고 70 KTS 이상에서 기수를 들어주세요.");
    return;
  }
  if (lesson.phase === "rollout" && flight.speed < 3) {
    flight.speed = 0;
    finishLesson(true, "활주로 안에 안전하게 접지하고 정지했습니다. 다음에는 중심선과 하강률을 더 일정하게 유지해보세요.");
  }
  flight.distance += flight.speed * dt * 0.0003;
}

function showMessage(text, error = false, duration = 0) {
  clearTimeout(messageTimer);
  const element = $("message");
  element.textContent = text;
  element.classList.toggle("error", error);
  element.hidden = false;
  if (duration) messageTimer = setTimeout(() => { element.hidden = true; }, duration);
}

// 기본 장치가 가상 카메라일 수 있으므로 실제 사용 중인 장치를 표시하고 선택하게 합니다.
async function refreshCameraList() {
  if (!navigator.mediaDevices?.enumerateDevices) return;
  try {
    const devices = (await navigator.mediaDevices.enumerateDevices()).filter(device => device.kind === "videoinput");
    const select = $("camera-select");
    const activeId = stream?.getVideoTracks()[0]?.getSettings().deviceId;
    const selectedId = activeId || select.value;
    const options = [new Option("브라우저 기본 카메라", "")];
    devices.forEach((device, index) => options.push(new Option(device.label || `카메라 ${index + 1}`, device.deviceId)));
    select.replaceChildren(...options);
    select.value = devices.some(device => device.deviceId === selectedId) ? selectedId : "";
  } catch (error) {
    // 목록 조회 실패만으로 정상적인 영상 스트림을 중단하지 않습니다.
    console.warn("카메라 장치 목록 조회 실패", error);
  }
}

// 1단계: 사용자 클릭 이후에만 카메라 권한을 요청합니다.
async function startCamera() {
  const button = $("camera-button");
  if (connectionStage === "camera" || connectionStage === "model") return;
  button.disabled = true;
  $("camera-select").disabled = true;
  button.textContent = "권한 확인 중…";
  connectionStage = "camera";
  let stage = "camera";
  try {
    if (window.location.protocol === "file:") {
      $("local-server-link").hidden = false;
      throw new Error("파일을 직접 열었습니다. 아래 ‘로컬 서버에서 비행 화면 열기’를 누르거나 VS Code Live Server로 실행해주세요.");
    }
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
      throw new Error("웹캠은 localhost 또는 HTTPS에서만 사용할 수 있습니다. VS Code Live Server로 열어주세요.");
    }
    showMessage("웹캠 권한을 허용해주세요.");
    const selectedId = $("camera-select").value;
    const videoConstraints = { width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30, max: 30 } };
    // exact를 사용해야 선택한 장치가 없을 때 엉뚱한 기본 카메라로 연결되지 않습니다.
    if (selectedId) videoConstraints.deviceId = { exact: selectedId };
    else videoConstraints.facingMode = "user";
    stream = await navigator.mediaDevices.getUserMedia({ video: videoConstraints, audio: false });
    video.srcObject = stream;
    await video.play();
    cameraActive = true;
    const connectedTrack = stream.getVideoTracks()[0];
    $("camera-device-status").textContent = `영상 입력: ${connectedTrack.label || "기본 카메라"}`;
    await refreshCameraList();
    $("camera-dot").classList.remove("off");
    connectedTrack.addEventListener("ended", () => {
      if (stream?.getVideoTracks()[0] === connectedTrack) {
        stopCamera();
        showMessage("카메라 연결이 끊어졌습니다. 웹캠을 다시 연결해주세요.", true);
      }
    });
    $("camera-placeholder").style.display = "none";
    const aspect = video.videoWidth / video.videoHeight;
    if (Number.isFinite(aspect)) document.querySelector(".camera-feed").style.aspectRatio = String(aspect);
    stage = "model";
    connectionStage = "model";
    button.textContent = "손 추적 준비 중…";
    showMessage("손 추적 모델을 불러오는 중입니다. 첫 연결은 잠시 걸릴 수 있습니다.");
    if (!landmarker) landmarker = await loadHandLandmarker();
    if (!cameraActive || connectedTrack.readyState === "ended") throw new Error("카메라 연결이 중단되었습니다.");
    cameraActive = true;
    connectionStage = "ready";
    calibration.note = "이지 조종은 손 위치를 자동으로 잡습니다. C 키는 R 표시 중심 보정용입니다.";
    lastVideoTime = -1;
    lastInference = 0;
    inferenceFailures = 0;
    $("camera-dot").classList.remove("off");
    button.textContent = "웹캠 끄기";
    showMessage("연결 완료. 오른손 위치와 관계없이 엄지와 검지를 붙이면 조종간을 잡습니다.", false, 7000);
  } catch (error) {
    if (stage === "model" && cameraActive) {
      // 손 추적 모델 실패를 웹캠 실패로 오해하지 않도록 영상은 유지합니다.
      connectionStage = "model-error";
      clearHands();
      button.textContent = "웹캠 끄기";
    } else stopCamera();
    console.error(`[${stage}]`, error);
    const messages = {
      NotAllowedError: "웹캠 권한이 거부되었습니다. 주소창의 사이트 권한에서 카메라를 허용한 뒤 다시 연결해주세요.",
      NotFoundError: "웹캠을 찾을 수 없습니다. 카메라 연결 상태를 확인해주세요.",
      NotReadableError: "웹캠을 열 수 없습니다. 다른 앱에서 카메라를 사용 중인지 확인해주세요.",
      OverconstrainedError: "선택한 카메라를 열 수 없습니다. 카메라 장치 목록에서 다른 웹캠을 선택해주세요.",
    };
    showMessage(stage === "model"
      ? `웹캠은 연결됐지만 손 추적 준비에 실패했습니다 (${error.message || error.name}). 인터넷/CDN 차단을 확인한 뒤 웹캠을 껐다 다시 연결해주세요.`
      : messages[error.name] || error.message || "웹캠 연결에 실패했습니다.", true);
  } finally {
    button.disabled = false;
    $("camera-select").disabled = false;
  }
}

// WASM 준비 단계에도 제한 시간을 적용합니다. 늦게 만들어진 모델은 즉시 정리합니다.
function withTimeout(promise, label, dispose) {
  let timer, expired = false;
  const guarded = promise.then(value => {
    if (expired) { dispose?.(value); throw new Error(`${label} 시간 초과`); }
    return value;
  });
  return Promise.race([guarded, new Promise((_, reject) => {
    timer = setTimeout(() => { expired = true; reject(new Error(`${label} 시간 초과`)); }, CDN_TIMEOUT);
  })]).finally(() => clearTimeout(timer));
}

// 2단계: JS와 WASM 버전을 고정하고 VIDEO / 두 손 추적을 사용합니다.
async function loadHandLandmarker() {
  let timer;
  const module = await Promise.race([
    import(VISION_URL),
    new Promise((_, reject) => { timer = setTimeout(() => reject(new Error("CDN 시간 초과")), CDN_TIMEOUT); }),
  ]).finally(() => clearTimeout(timer));
  const fileset = await withTimeout(module.FilesetResolver.forVisionTasks(WASM_URL), "WASM 준비");
  const abort = new AbortController();
  const modelTimer = setTimeout(() => abort.abort(), CDN_TIMEOUT);
  let modelBytes;
  try {
    const response = await fetch(MODEL_URL, { signal: abort.signal });
    if (!response.ok) throw new Error(`모델 응답: ${response.status}`);
    modelBytes = new Uint8Array(await response.arrayBuffer());
  } finally { clearTimeout(modelTimer); }
  const options = {
    baseOptions: { modelAssetBuffer: modelBytes, delegate: "GPU" },
    runningMode: "VIDEO", numHands: 2,
    minHandDetectionConfidence: 0.55, minHandPresenceConfidence: 0.55, minTrackingConfidence: 0.55,
  };
  try { return await withTimeout(module.HandLandmarker.createFromOptions(fileset, options), "GPU 모델 초기화", model => model.close()); }
  catch (gpuError) {
    // 시간 초과한 초기화는 백그라운드에 남을 수 있어 CPU 작업을 중복 시작하지 않습니다.
    if (gpuError.message.includes("시간 초과")) throw gpuError;
    // GPU 초기화 실패 시 CPU로 한 번 재시도합니다.
    console.warn("GPU 사용 불가, CPU로 재시도합니다.", gpuError);
    options.baseOptions.delegate = "CPU";
    return await withTimeout(module.HandLandmarker.createFromOptions(fileset, options), "CPU 모델 초기화", model => model.close());
  }
}

function releaseStick() {
  stick.grabbed = false;
  stick.anchor = null;
  stick.depthAnchor = null;
  stick.depthShape = null;
}
function resetCalibrationSamples() {
  Object.assign(calibration, { held: 0, lastSample: 0, reference: null, referenceScale: null, sumScale: 0, sumShape: 0, sumX: 0, sumY: 0, count: 0 });
}
function cancelCalibration() {
  if (!calibration.active) return;
  calibration.active = false;
  resetCalibrationSamples();
  calibration.note = calibration.neutral ? "설정을 취소했습니다. 이전 중심을 유지합니다." : "설정을 취소했습니다. 기본 중심을 사용합니다.";
}
function clearCalibration() {
  cancelCalibration();
  calibration.neutral = null;
  calibration.palmScale = null; calibration.palmShape = null;
  calibration.note = "표시 중심 보정을 해제했습니다. 화면 어디서든 핀치해 조종할 수 있습니다.";
  releaseStick();
}
function startCalibration() {
  if (!cameraActive || connectionStage !== "ready") return;
  releaseStick();
  resetCalibrationSamples();
  calibration.active = true;
  calibration.started = performance.now();
  calibration.note = "오른손의 엄지·검지를 살짝 벌리고 편한 자세를 잡으세요.";
  showMessage("2초 뒤 측정합니다. 오른손의 엄지·검지를 살짝 벌리고 편하게 유지하세요.", false, 4000);
}
// 원래 영상 좌표를 보관한 채 R 커서만 보정하므로 미리보기 랜드마크는 왜곡되지 않습니다.
function controlPoint(point) {
  if (!calibration.neutral) return point;
  const factor = depthReady(hands.Right, calibration.palmScale) ? clamp(calibration.palmScale/hands.Right.palmScale, .5, 1.8) : 1;
  // 앞뒤로 움직일 때 원근 때문에 손의 화면 좌표도 달라지는 현상을 완화합니다.
  return { x: .5 + (point.x-.5)*factor - calibration.neutral.x + STICK_CENTER.x, y: .5 + (point.y-.5)*factor - calibration.neutral.y + STICK_CENTER.y };
}

function verticalPitchInput(hand) {
  if (!hand.detected || !hand.point) return 0;
  const neutralY = stick.anchor?.y ?? calibration.neutral?.y;
  if (!Number.isFinite(neutralY)) return 0;
  const verticalOffset = (hand.point.y-neutralY)/VERTICAL_PITCH_RANGE;
  // 이지 모드는 화면에서 손을 위로 올리는 동작을 상승으로 연결합니다.
  // 고급 모드의 선택형 위아래 보조는 기존 조종간 방향을 유지합니다.
  return clamp(easyControlEnabled() ? -verticalOffset : verticalOffset,-1,1);
}

function easyControlEnabled() { return $("control-mode").value === "easy"; }
function controlResponse() { return CONTROL_RESPONSE_PRESETS[$("control-response")?.value] || CONTROL_RESPONSE_PRESETS.fast; }
function appliedStickInput() {
  const gain=controlResponse().gain;
  return {x:clamp(stick.x*gain,-1,1),y:clamp(stick.y*gain,-1,1)};
}

function pitchControlInput(hand) {
  const vertical = verticalPitchInput(hand);
  if (easyControlEnabled()) return vertical;
  const verticalAssist = $("vertical-pitch-assist").checked;
  const referenceScale = stick.grabbed ? stick.depthAnchor : calibration.palmScale;
  if (!referenceScale || !depthReady(hand, referenceScale)) return verticalAssist ? vertical : 0;
  const depth = depthPitchInput(hand, referenceScale);
  // 기본값은 실제 조종간과 같은 앞뒤 입력만 사용합니다. 사용자가 보조 옵션을 켠 경우만 합칩니다.
  return clamp(depth + (verticalAssist ? vertical*VERTICAL_PITCH_ASSIST : 0),-1,1);
}
function updateCalibration(now) {
  if (!calibration.active) return;
  if (now - calibration.started > CALIBRATION_TIMEOUT_MS) {
    cancelCalibration();
    calibration.note = "설정 시간이 초과되었습니다. 오른손을 보여준 뒤 다시 시작하세요.";
    return;
  }
  if (now - calibration.started < CALIBRATION_PREPARE_MS) return;
  const hand = hands.Right;
  if (!hand.detected || !hand.point || hand.pinch || !hand.depthValid) {
    resetCalibrationSamples();
    calibration.note = "손바닥이 잘 보이게 하고 엄지·검지를 살짝 벌린 채 거리도 유지하세요.";
    return;
  }
  const point = hand.point;
  if (calibration.reference && (now - calibration.lastSample > HAND_LOST_TIMEOUT || Math.hypot(point.x-calibration.reference.x, point.y-calibration.reference.y) > CALIBRATION_STABILITY || Math.abs(hand.palmScale/calibration.referenceScale-1) > 0.06)) {
    resetCalibrationSamples();
  }
  if (!calibration.reference) { calibration.reference = { ...point }; calibration.referenceScale = hand.palmScale; }
  if (calibration.lastSample) calibration.held += now - calibration.lastSample;
  calibration.lastSample = now;
  calibration.sumX += point.x; calibration.sumY += point.y; calibration.count++;
  calibration.sumScale += hand.palmScale; calibration.sumShape += hand.palmShape;
  calibration.note = `편한 위치에서 움직이지 마세요 · ${Math.max(1, Math.ceil((CALIBRATION_HOLD_MS-calibration.held)/1000))}초`;
  if (calibration.held >= CALIBRATION_HOLD_MS) {
    calibration.neutral = { x: calibration.sumX/calibration.count, y: calibration.sumY/calibration.count };
    calibration.palmScale = calibration.sumScale/calibration.count;
    calibration.palmShape = calibration.sumShape/calibration.count;
    calibration.active = false;
    releaseStick();
    calibration.note = "위치·거리 설정 완료 · 핀치 후 몸 쪽으로 당기면 상승, 카메라 쪽으로 밀면 하강합니다.";
    showMessage("거리 기준 설정 완료! 핀치한 뒤 몸 쪽으로 당기면 기수가 올라갑니다.", false, 6000);
  }
}
function clearHands() {
  cancelCalibration();
  Object.assign(hands.Right, newHand());
  Object.assign(hands.Left, newHand());
  releaseStick();
  landmarkCtx.clearRect(0, 0, landmarkCanvas.width, landmarkCanvas.height);
}
function stopCamera() {
  if (lesson.mode !== "free" && !lesson.result) lesson.paused = true;
  cameraActive = false;
  connectionStage = "idle";
  stream?.getTracks().forEach(track => track.stop());
  stream = null;
  video.srcObject = null;
  clearHands();
  clearCalibration();
  $("camera-dot").classList.add("off");
  $("camera-placeholder").style.display = "flex";
  $("camera-button").textContent = "웹캠 연결 ↗";
  $("camera-device-status").textContent = "카메라 연결이 꺼져 있습니다.";
}

// 화면에 보여주는 좌표는 거울 반전. 비디오와 랜드마크 모두 같은 변환입니다.
function handPoint(landmarks) {
  // 손가락을 벌렸다 핀치해도 기준점이 움직이지 않도록 손바닥 중심을 사용합니다.
  const ids = [0,5,9,13,17];
  const x = ids.reduce((sum,index)=>sum+landmarks[index].x,0)/ids.length;
  const y = ids.reduce((sum,index)=>sum+landmarks[index].y,0)/ids.length;
  return { x: 1-x, y };
}
// MediaPipe z는 손목 기준 상대 좌표여서 카메라까지의 거리로 사용할 수 없습니다.
// 손가락 끝 대신 손바닥 뼈대 크기로 상대 거리를 추정합니다. 절대 cm 값은 아닙니다.
function measurePalm(landmarks, aspect) {
  const ids = [0,5,9,13,17];
  // 화면 가장자리에서도 좌표가 유효하면 측정을 유지합니다.
  // 이전 범위 검사는 손목이 가장자리에 닿는 순간 거리 입력 전체를 꺼 버렸습니다.
  if (!ids.every(i => Number.isFinite(landmarks[i]?.x) && Number.isFinite(landmarks[i]?.y))) return null;
  const distance = (a,b) => Math.hypot((landmarks[a].x-landmarks[b].x)*aspect,landmarks[a].y-landmarks[b].y);
  const palmWidth = distance(5,17);
  const palmLength = (distance(0,5)+distance(0,9)+distance(0,13)+distance(0,17))/4;
  if (palmWidth < .015 || palmLength < .015) return null;
  return { scale: Math.sqrt(palmWidth*palmLength), shape: palmWidth/palmLength };
}
function depthReady(hand, referenceScale = calibration.palmScale) {
  return hand.detected && hand.depthValid && hand.palmScale > 0 && referenceScale > 0;
}
function palmShapeStable(hand, referenceShape = stick.grabbed ? stick.depthShape : calibration.palmShape) {
  return depthReady(hand, stick.grabbed ? stick.depthAnchor : calibration.palmScale) && referenceShape > 0
    && Math.abs(hand.palmShape/referenceShape-1) <= DEPTH_SHAPE_TOLERANCE;
}
function distanceChange(hand, referenceScale = stick.grabbed ? stick.depthAnchor : calibration.palmScale) {
  if (!depthReady(hand, referenceScale)) return 0;
  // 로그 비율을 쓰면 카메라 쪽/몸 쪽 움직임의 민감도가 대칭이 됩니다.
  // 양수: 손이 작아짐(몸 쪽으로 당김), 음수: 손이 커짐(카메라 쪽으로 밀기).
  return Math.log(referenceScale/hand.palmScale);
}
function depthPitchInput(hand, referenceScale = stick.grabbed ? stick.depthAnchor : calibration.palmScale) {
  if (!depthReady(hand, referenceScale)) return 0;
  const relativeDistance = distanceChange(hand, referenceScale);
  const magnitude = Math.max(0, Math.abs(relativeDistance)-DEPTH_DEADZONE)/(DEPTH_RANGE-DEPTH_DEADZONE);
  return Math.sign(relativeDistance)*clamp(magnitude,0,1);
}
function pinchRatio(landmarks, aspect) {
  // 영상의 종횡비를 보정해야 가로와 세로 거리를 공정하게 비교할 수 있습니다.
  const distance = (a, b) => Math.hypot((a.x - b.x) * aspect, a.y - b.y);
  return distance(landmarks[4], landmarks[8]) / Math.max(0.015, distance(landmarks[5], landmarks[17]));
}
// 3~4단계: 핀치 히스테리시스 → 현재 손 위치/거리를 중립점으로 잡아 상대 이동 입력.
function processHands(result, now) {
  const selected = {};
  const swap = $("swap-hands").checked;
  const aspect = video.videoWidth / video.videoHeight || 4 / 3;
  const candidates = result.landmarks.map((landmarks, index) => {
    const category = result.handedness?.[index]?.[0];
    let side = category?.categoryName;
    if (side !== "Left" && side !== "Right") side = null;
    if (swap && side) side = side === "Left" ? "Right" : "Left";
    return { landmarks, score: category?.score ?? 0, side, point: handPoint(landmarks) };
  });
  if (candidates.length) {
    // MediaPipe의 순간적인 좌우 판정보다 기존 손 위치의 연속성을 우선합니다.
    // 한 손이 사라져도 남은 손을 빈 반대쪽 역할로 승격하지 않습니다.
    const trackingCost = (side, candidate) => {
      const previous = hands[side];
      const labelCost = !candidate.side ? 0.08 : candidate.side === side ? -0.05 : 0.22;
      if (!previous.detected || !previous.point) return 0.28 + labelCost;
      return Math.hypot(candidate.point.x-previous.point.x,candidate.point.y-previous.point.y) + labelCost;
    };
    if (candidates.length === 1) {
      const candidate = candidates[0];
      const rightCost = trackingCost("Right",candidate);
      const leftCost = trackingCost("Left",candidate);
      selected[rightCost <= leftCost ? "Right" : "Left"] = candidate;
    } else {
      const direct = trackingCost("Right",candidates[0]) + trackingCost("Left",candidates[1]);
      const crossed = trackingCost("Right",candidates[1]) + trackingCost("Left",candidates[0]);
      if (direct <= crossed) {
        selected.Right = candidates[0]; selected.Left = candidates[1];
      } else {
        selected.Right = candidates[1]; selected.Left = candidates[0];
      }
    }
  }
  for (const side of ["Right", "Left"]) {
    const hand = hands[side];
    const match = selected[side];
    if (!match) {
      // 핀치하면 손가락이 겹쳐 한두 프레임 검출이 빠질 수 있으므로 잠깐 유지합니다.
      if (hand.detected && now-hand.lastSeen <= HAND_LOST_TIMEOUT) continue;
      Object.assign(hand, newHand());
      if (side === "Right") releaseStick();
      continue;
    }
    const raw = handPoint(match.landmarks);
    const dt = hand.lastSeen ? Math.min((now - hand.lastSeen) / 1000, 0.1) : 1 / 30;
    const alpha = 1 - Math.pow(1 - SMOOTHING_FACTOR, dt * 30);
    hand.point = hand.point ? { x: lerp(hand.point.x, raw.x, alpha), y: lerp(hand.point.y, raw.y, alpha) } : raw;
    hand.detected = true;
    hand.lastSeen = now;
    const palm = measurePalm(match.landmarks, aspect);
    hand.depthValid = Boolean(palm);
    if (palm) {
      const depthAlpha = 1-Math.pow(1-DEPTH_SMOOTHING,dt*30);
      hand.palmScale = hand.palmScale ? lerp(hand.palmScale,palm.scale,depthAlpha) : palm.scale;
      hand.palmShape = palm.shape;
    } else { hand.palmScale = null; hand.palmShape = null; }
    const ratio = pinchRatio(match.landmarks, aspect);
    hand.pinch = ratio < (hand.pinch ? PINCH_RELEASE_THRESHOLD : PINCH_THRESHOLD);
    if (side === "Right") {
      const wantsControl = easyControlEnabled() || hand.pinch;
      if (!wantsControl) releaseStick();
      else if (!calibration.active && !stick.grabbed) {
        stick.grabbed = true;
        // 이지 모드는 손이 나타난 위치, 거리 모드는 핀치 시작 위치를 중립으로 사용합니다.
        stick.anchor = { ...hand.point };
        stick.depthAnchor = hand.depthValid ? hand.palmScale : null;
        stick.depthShape = hand.depthValid ? hand.palmShape : null;
      }
    }
  }
  updateCalibration(now);
  drawLandmarks(selected);
}

function trackHands(now) {
  if (!cameraActive || connectionStage !== "ready" || !landmarker || video.readyState < 2 || document.hidden) return;
  if (now - lastInference < 1000 / TRACKING_FPS || video.currentTime === lastVideoTime) return;
  lastInference = now;
  lastVideoTime = video.currentTime;
  try {
    processHands(landmarker.detectForVideo(video, now), now);
    inferenceFailures = 0;
  } catch (error) {
    clearHands();
    if (++inferenceFailures >= 3) {
      console.error("손 추적 실패", error);
      stopCamera();
      landmarker.close(); landmarker = null;
      showMessage("손 추적 중 오류가 발생했습니다. 웹캠 연결을 눌러 다시 시도해주세요.", true);
    }
  }
}

function drawLandmarks(selected) {
  // 캔버스 해상도를 입력 영상과 맞춰 비율 왜곡이나 이중 반전을 피합니다.
  if (landmarkCanvas.width !== video.videoWidth || landmarkCanvas.height !== video.videoHeight) {
    landmarkCanvas.width = video.videoWidth; landmarkCanvas.height = video.videoHeight;
  }
  const w = landmarkCanvas.width, h = landmarkCanvas.height;
  landmarkCtx.clearRect(0, 0, w, h);
  for (const [side, { landmarks }] of Object.entries(selected)) {
    const color = side === "Right" ? "#bcf7b0" : "#91dcff";
    landmarkCtx.strokeStyle = color; landmarkCtx.fillStyle = color; landmarkCtx.lineWidth = 2;
    for (const [a, b] of CONNECTIONS) {
      landmarkCtx.beginPath();
      landmarkCtx.moveTo((1 - landmarks[a].x) * w, landmarks[a].y * h);
      landmarkCtx.lineTo((1 - landmarks[b].x) * w, landmarks[b].y * h);
      landmarkCtx.stroke();
    }
    for (const p of landmarks) {
      landmarkCtx.beginPath(); landmarkCtx.arc((1 - p.x) * w, p.y * h, 3, 0, Math.PI * 2); landmarkCtx.fill();
    }
    landmarkCtx.font = "bold 18px monospace";
    landmarkCtx.fillText(side.toUpperCase(), clamp((1 - landmarks[0].x) * w, 4, w - 70), clamp(landmarks[0].y * h + 24, 20, h - 6));
  }
}

// 5~6, 8단계: 입력, 자세, 속도와 고도를 시간 기반으로 갱신합니다.
function updateFlight(dt, now) {
  if (calibration.active && now - calibration.started > CALIBRATION_TIMEOUT_MS) updateCalibration(now);
  for (const [side, hand] of Object.entries(hands)) {
    if (hand.detected && now - hand.lastSeen > HAND_LOST_TIMEOUT) {
      Object.assign(hand, newHand());
      landmarkCtx.clearRect(0, 0, landmarkCanvas.width, landmarkCanvas.height);
      if (side === "Right") releaseStick();
    }
  }
  const right = hands.Right;
  if (stick.grabbed && right.detected && right.point && stick.anchor) {
    // 핀치를 시작한 지점이 항상 중립이므로 화면 어디에서 잡아도 Roll이 튀지 않습니다.
    const lateralOffset = right.point.x-stick.anchor.x;
    stick.x = clamp(lateralOffset/STICK_RANGE,-1,1);
    // 핀치 순간 크기 측정이 한 프레임 늦은 경우 첫 유효 프레임에서 거리 중립점을 잡습니다.
    if (!stick.depthAnchor && right.depthValid) {
      stick.depthAnchor = right.palmScale;
      stick.depthShape = right.palmShape;
    }
    stick.y = damp(stick.y,pitchControlInput(right),controlResponse().inputSpeed,dt);
  } else {
    stick.x = damp(stick.x, 0, STICK_RETURN_SPEED, dt);
    stick.y = damp(stick.y, 0, STICK_RETURN_SPEED, dt);
  }
  // 훈련 시작 화면에서 사용자가 실제로 조종간을 잡고 움직이면 이를 시작 의도로 봅니다.
  // 입력 수치는 변하지만 PITCH는 멈춰 있던 혼란스러운 상태를 없앱니다.
  const controlStartRequested = easyControlEnabled()
    ? Math.max(Math.abs(stick.x),Math.abs(stick.y)) >= 0.05
    : stick.grabbed;
  if (lesson.mode !== "free" && lesson.paused && !lesson.result && !calibration.active
      && (controlStartRequested || flight.throttle >= 10)) {
    lesson.paused = false;
    showMessage("손 조종 입력을 감지해 비행을 시작했습니다.",false,3500);
  }
  const left = hands.Left;
  if (!calibration.active && left.detected && left.point) {
    const target = clamp((THROTTLE_BOTTOM - left.point.y) / (THROTTLE_BOTTOM - THROTTLE_TOP), 0, 1) * 100;
    flight.throttle = damp(flight.throttle, target, 3.5, dt);
  }
  // 일시정지 중에도 손 추적과 중심 설정은 계속합니다. 위치·자세만 고정합니다.
  if (lesson.paused || lesson.result || calibration.active) return;
  if (lesson.mode !== "free") { updateRunwayFlight(dt); applyWeatherFlight(dt,now); return; }
  // 양수 pitch는 기수 상승. 놓으면 초보자가 회복하기 쉬운 수평 자세로 복귀합니다.
  const response=controlResponse();
  const applied=appliedStickInput();
  flight.roll = damp(flight.roll, applied.x * MAX_ROLL, response.rollSpeed, dt);
  flight.pitch = damp(flight.pitch, applied.y * MAX_PITCH, response.pitchSpeed, dt);
  flight.speed = damp(flight.speed, 45 + flight.throttle * 2.2 - flight.pitch * 0.55, 0.35, dt);
  flight.verticalSpeed = Math.sin(radians(flight.pitch)) * flight.speed * 1.68781 * 0.48; // ft/s
  if (flight.altitude <= 0 && flight.verticalSpeed < 0) flight.verticalSpeed = 0;
  flight.altitude = Math.max(0, flight.altitude + flight.verticalSpeed * dt);
  flight.heading = (flight.heading + Math.sin(radians(flight.roll)) * flight.speed * 0.06 * dt + 360) % 360;
  // 자유 비행에서도 지형 좌표를 실제 속도로 이동시켜 도시와 도로가 아래로 지나갑니다.
  const direction=radians(flight.heading-RUNWAY.heading),travel=flight.speed*KNOTS_TO_MPS*dt;
  lesson.x+=Math.sin(direction)*travel;lesson.z+=Math.cos(direction)*travel;
  flight.distance += flight.speed * dt * 0.0003;
  applyWeatherFlight(dt,now);
}

// 악천후에서도 손 조종을 방해하지 않는 범위로 약한 돌풍만 물리에 반영합니다.
function applyWeatherFlight(dt,now) {
  const weather=WEATHER_PRESETS[weatherMode];
  if(!weather||weather.wind<=0||lesson.phase!=="airborne")return;
  const gust=Math.sin(now*.0017)+Math.sin(now*.0041+.8)*.45;
  flight.heading=(flight.heading+gust*weather.wind*.7*dt+360)%360;
  flight.roll=clamp(flight.roll+gust*weather.wind*1.8*dt,-MAX_ROLL,MAX_ROLL);
  lesson.x+=Math.sin(now*.00041+1.2)*weather.wind*1.4*dt;
}

// 속도 경계에 서로 다른 진입/해제 값을 사용해 경고가 빠르게 깜빡이지 않게 합니다.
function updateFlightEffects(now) {
  const canStall = !lesson.paused && !lesson.result && lesson.phase === "airborne" && flight.altitude > 15;
  if (!canStall) effects.stall=false;
  else if (!effects.stall && flight.speed < STALL_ENTER_SPEED) effects.stall=true;
  else if (effects.stall && flight.speed > STALL_EXIT_SPEED) effects.stall=false;

  $("flight-warning").hidden=!effects.stall;
  if (effects.stall && now-effects.lastStallTone>850) {
    effects.lastStallTone=now;
    playTone(185,.13,.12,0,"square");
    playTone(135,.16,.10,.16,"square");
  }
}

// 7단계: Canvas 2D 배경만 움직여 1인칭 시야를 만듭니다. 실제 3D 엔진은 없습니다.
function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  width = rect.width; height = rect.height;
  dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
  // 크기가 바뀌는 도중 의도치 않은 조종 입력을 방지합니다.
  releaseStick();
}
function path(points, fill, stroke) {
  ctx.beginPath(); points.forEach(([x,y], i) => i ? ctx.lineTo(x,y) : ctx.moveTo(x,y));
  ctx.closePath(); if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.strokeStyle = stroke; ctx.stroke(); }
}
function drawWorld(now) {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if(realEarth.enabled){drawRealEarthCanvas(now);return;}
  ctx.save();
  ctx.translate(width / 2, height * 0.46);
  // 오른쪽으로 기울이면 외부 수평선은 반시계 방향으로 회전합니다.
  ctx.rotate(-radians(flight.roll));
  ctx.translate(0, flight.pitch * height * 0.012);
  const extent = Math.hypot(width, height) * 2;
  const theme=MAP_THEMES[worldMap];
  const weather=WEATHER_PRESETS[weatherMode];
  const terrainReady=Boolean(terrainTextures[worldMap]?.complete&&terrainTextures[worldMap]?.naturalWidth);
  const skyColors=weather.sky||theme.sky;
  const sky = ctx.createLinearGradient(0, -height, 0, 20);
  sky.addColorStop(0,skyColors[0]);sky.addColorStop(.65,skyColors[1]);sky.addColorStop(1,skyColors[2]);
  ctx.fillStyle = sky; ctx.fillRect(-extent, -extent, extent * 2, extent);
  const ground = ctx.createLinearGradient(0, 0, 0, height);
  ground.addColorStop(0,theme.ground[0]);ground.addColorStop(.18,theme.ground[1]);ground.addColorStop(1,theme.ground[2]);
  ctx.fillStyle = ground; ctx.fillRect(-extent, 0, extent * 2, extent);

  drawSkyDetails(extent,theme,now,weather);
  if(theme.night&&weatherMode==="clear") drawNightSky(extent);
  if(theme.mountain) drawMountainRanges(extent,theme);
  if(!terrainReady&&(worldMap==="mountain-city"||worldMap==="night-city")) drawDistantSkyline(extent,theme);

  drawCloudField(theme,now,weather);
  drawTerrainImage(theme,extent);
  if(terrainReady) drawWorldDetailOverlay(theme);
  drawRegionalGround(theme);
  drawTerrainTexture(theme);

  // 지면의 원근 격자와 패치가 전진감을 줍니다. 고도에 따라 격자 크기도 완만히 변합니다.
  const scale = clamp(2400 / (flight.altitude + 400), 0.35, 2);
  const drift = Math.sin(radians(flight.heading - 300)) * width * 0.7;
  const texturedGround=terrainReady;
  // 실제 조종석 시야처럼 지면 격자는 거의 보이지 않게 두고 빠른 이동감만 남깁니다.
  ctx.lineWidth = 1; ctx.strokeStyle = `rgba(${theme.grid},${texturedGround?.008:.035})`;
  for (let i = -16; i <= 16; i++) {
    ctx.beginPath(); ctx.moveTo(i * 20 + drift * 0.07, 0); ctx.lineTo(i * width * 0.21 + drift, extent); ctx.stroke();
  }
  for (let i = 0; i < 19; i++) {
    // 속도에 비례해 격자가 조종석 쪽으로 흘러 지상에서도 가속감을 읽을 수 있습니다.
    const depth = ((i / 19 + flight.distance * 2.4) % 1);
    const y = depth * depth * height * 1.65 * scale;
    ctx.strokeStyle = `rgba(${theme.grid},${depth*(texturedGround?.01:.045)})`;
    ctx.beginPath(); ctx.moveTo(-extent, y); ctx.lineTo(extent, y); ctx.stroke();
  }
  // 텍스처 지형과 예전 절차 지형을 동시에 표시하지 않습니다. 로딩 실패 때만 절차 지형을 대체 화면으로 사용합니다.
  if(!terrainReady) {
    if(worldMap==="ocean-islands") drawOceanScenery(); else drawCityScenery(theme);
  }
  if (lesson.mode !== "free") drawRunway();
  if (lesson.mode === "mission" && !mission.returning) drawCheckpointRings();
  // 먼 수평선: 기울기와 피치 방향을 쉽게 읽을 수 있는 얇은 빛.
  ctx.strokeStyle = "#d7e8cb77"; ctx.beginPath(); ctx.moveTo(-extent,0); ctx.lineTo(extent,0); ctx.stroke();
  drawPitchLadder();
  ctx.restore();
  drawMapColorGrade(theme);
  drawWeatherEffects(now,weather);
  drawWindStreaks(now);
  if (lesson.mode === "mission" && !mission.returning) drawCheckpointNavigator();
  if (lesson.mode === "mission") drawGatePassEffect(now);
}

// 실제 지형 모드에서는 배경을 칠하지 않고 기존 Canvas의 훈련 표식과 날씨 효과만 투명하게 겹칩니다.
function drawRealEarthCanvas(now) {
  // 밝은 지도 위에서도 연녹색 HUD가 읽히도록 조종석 유리와 같은 약한 청록 음영을 겹칩니다.
  ctx.save();
  const glass=ctx.createLinearGradient(0,0,0,height);
  glass.addColorStop(0,worldMap==="night-city"?"rgba(2,9,22,.58)":"rgba(2,13,18,.16)");
  glass.addColorStop(.65,worldMap==="night-city"?"rgba(2,9,22,.48)":"rgba(2,15,19,.24)");
  glass.addColorStop(1,"rgba(1,10,15,.34)");ctx.fillStyle=glass;ctx.fillRect(0,0,width,height);ctx.restore();
  ctx.save();ctx.translate(width/2,height*.46);ctx.rotate(-radians(flight.roll));
  ctx.translate(0,flight.pitch*height*.012);
  if(lesson.mode!=="free")drawRunway();
  if(lesson.mode==="mission"&&!mission.returning)drawCheckpointRings();
  drawPitchLadder();ctx.restore();
  drawWeatherEffects(now,WEATHER_PRESETS[weatherMode]);
  drawWindStreaks(now);
  if(lesson.mode==="mission"&&!mission.returning)drawCheckpointNavigator();
  if(lesson.mode==="mission")drawGatePassEffect(now);
}

// 여러 개의 반투명 타원과 음영을 겹쳐 납작한 구름 대신 부피 있는 구름층을 만듭니다.
function drawCloudField(theme,now,weather) {
  const band=width*3.2;
  ctx.save();
  const cloudCount=Math.min(18,Math.round(11*weather.clouds));
  for(let i=0;i<cloudCount;i++) {
    const phase=((i*347+now*(.0014+(i%3)*.00035)+flight.heading*5.6)%band)-band/2;
    const y=-62-(i%4)*height*.145-sceneryNoise(i*4.8)*28;
    const size=20+(i%4)*9;
    const baseAlpha=weather.overcast?.16:theme.night?.035:.075;
    const alpha=baseAlpha+(i%3)*(weather.overcast?.035:theme.night?.013:.026);
    const shadow=ctx.createRadialGradient(phase,y+size*.22,size*.1,phase,y,size*2.8);
    shadow.addColorStop(0,`rgba(${theme.cloud},${alpha*1.35})`);
    shadow.addColorStop(.55,`rgba(${theme.cloud},${alpha})`);
    shadow.addColorStop(1,`rgba(${theme.cloud},0)`);
    ctx.fillStyle=shadow;ctx.beginPath();ctx.ellipse(phase,y,size*3.6,size*.78,0,0,Math.PI*2);ctx.fill();
    for(let puff=0;puff<4;puff++) {
      const px=phase+(puff-1.5)*size*.82,py=y-size*(.08+sceneryNoise(i*7+puff)*.3);
      const pr=size*(.72+sceneryNoise(i*13+puff)*.48);
      const glow=ctx.createRadialGradient(px-pr*.2,py-pr*.3,0,px,py,pr);
      glow.addColorStop(0,`rgba(${theme.cloud},${alpha*1.8})`);glow.addColorStop(1,`rgba(${theme.cloud},0)`);
      ctx.fillStyle=glow;ctx.beginPath();ctx.ellipse(px,py,pr*1.35,pr*.72,0,0,Math.PI*2);ctx.fill();
    }
  }
  ctx.restore();
}

// 화면 가장자리의 은은한 감광과 수평선 산란광으로 깊이와 명암을 정리합니다.
function drawMapColorGrade(theme) {
  ctx.save();
  const horizon=height*.46+flight.pitch*height*.012;
  const bloom=ctx.createLinearGradient(0,horizon-height*.18,0,horizon+height*.24);
  bloom.addColorStop(0,"#e9f0dc00");
  bloom.addColorStop(.46,theme.night?"#79a2ad10":theme.desert?"#ffd69a20":"#e7eed51a");
  bloom.addColorStop(1,"#07131b00");ctx.fillStyle=bloom;ctx.fillRect(0,0,width,height);
  const vignette=ctx.createRadialGradient(width*.5,height*.43,height*.18,width*.5,height*.43,Math.max(width,height)*.72);
  vignette.addColorStop(.45,"#00000000");vignette.addColorStop(1,theme.night?"#00081273":"#07151b45");
  ctx.fillStyle=vignette;ctx.fillRect(0,0,width,height);
  ctx.restore();
}

function drawWeatherEffects(now,weather) {
  ctx.save();
  if(weather.overcast) {
    ctx.fillStyle=weather.rain?"#07172235":"#32444d20";ctx.fillRect(0,0,width,height);
  }
  if(weather.sunset) {
    const glow=ctx.createRadialGradient(width*.76,height*.39,0,width*.76,height*.39,width*.42);
    glow.addColorStop(0,"#ffb25b23");glow.addColorStop(.5,"#d76b4a12");glow.addColorStop(1,"#532f4b00");
    ctx.fillStyle=glow;ctx.fillRect(0,0,width,height);
  }
  if(weather.haze>0) {
    const horizon=height*.46+flight.pitch*height*.012;
    const fog=ctx.createLinearGradient(0,horizon-height*.3,0,height);
    fog.addColorStop(0,"#d6dfdc00");fog.addColorStop(.34,`rgba(199,211,208,${weather.haze*.7})`);
    fog.addColorStop(1,`rgba(166,181,179,${weather.haze*(weather.fog?.92:.28)})`);
    ctx.fillStyle=fog;ctx.fillRect(0,0,width,height);
  }
  if(weather.rain) {
    const gust=Math.sin(now*.0017)*13;
    ctx.lineWidth=1;ctx.lineCap="round";
    for(let i=0;i<105;i++) {
      const seed=sceneryNoise(i*9.17),speed=.42+sceneryNoise(i*4.31)*.72;
      const x=(sceneryNoise(i*2.73)*width+now*speed*.18+gust*i*.03)%(width+100)-50;
      const y=(sceneryNoise(i*7.51)*height+now*speed*.52)%(height+90)-45;
      const length=10+speed*22;
      ctx.strokeStyle=`rgba(205,230,235,${.12+speed*.25})`;
      ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x-4-gust*.08,y+length);ctx.stroke();
    }
    // 유리 표면의 큰 물방울은 가장자리만 그려 HUD 판독성을 유지합니다.
    ctx.strokeStyle="#d9edf136";ctx.lineWidth=1.1;
    for(let i=0;i<14;i++) {
      const phase=(now*.000045*(.7+i%4)+sceneryNoise(i)*1.3)%1;
      const x=sceneryNoise(i*14.1)*width,y=(sceneryNoise(i*8.3)*height+phase*height*.28)%height;
      const r=2+sceneryNoise(i*3.9)*5;
      ctx.beginPath();ctx.arc(x,y,r,.15,Math.PI*1.55);ctx.stroke();
    }
    const flash=Math.pow(Math.max(0,Math.sin(now*.00023*7.3)-.992)*125,2);
    if(flash>0) {ctx.fillStyle=`rgba(220,235,240,${Math.min(.22,flash*.08)})`;ctx.fillRect(0,0,width,height);}
  }
  if(weather.fog) {
    // 서로 다른 속도의 안개 띠가 천천히 흘러 정적인 흰 막처럼 보이지 않게 합니다.
    for(let i=0;i<5;i++) {
      const x=((now*.004*(i+1)+i*311)%(width*1.7))-width*.35;
      const y=height*(.28+i*.12),rx=width*(.25+i*.035);
      const mist=ctx.createRadialGradient(x,y,0,x,y,rx);
      mist.addColorStop(0,"#e0e7e351");mist.addColorStop(1,"#d9e1de00");
      ctx.fillStyle=mist;ctx.beginPath();ctx.ellipse(x,y,rx,height*.13,0,0,Math.PI*2);ctx.fill();
    }
  }
  ctx.restore();
}

function sceneryNoise(value) {
  const wave=Math.sin(value*12.9898+78.233)*43758.5453;
  return wave-Math.floor(wave);
}

// 한 장의 항공 지형 텍스처를 여러 가로 띠로 잘라 간단한 원근 지면으로 만듭니다.
// Canvas 2D만 사용하지만 수평선 쪽 띠는 좁고 가까운 띠는 넓어져 비행 중 지면이 아래로 흐릅니다.
function drawTerrainImage(theme,extent) {
  // 선택한 맵만 내려받아 첫 접속에서 네 장의 큰 이미지를 동시에 요청하지 않습니다.
  const image=terrainTexture(worldMap);
  if(!image||!image.complete||!image.naturalWidth)return;
  const slices=28,sourceHeight=image.naturalHeight/slices;
  // 별도 애니메이션 값을 섞지 않고 실제 월드 위치에만 고정합니다. 전진하면 지형 특징이 조종석 방향으로 이동합니다.
  const forward=((-lesson.z*.16)%image.naturalHeight+image.naturalHeight)%image.naturalHeight;
  const lateral=((lesson.x*.13)%image.naturalWidth+image.naturalWidth)%image.naturalWidth;
  const horizontalPhase=lateral/image.naturalWidth;
  ctx.save();
  ctx.imageSmoothingEnabled=true;ctx.imageSmoothingQuality="high";
  const baseAlpha=worldMap==="night-city"?.72:worldMap==="ocean-islands"?.76:.70;
  ctx.filter=theme.night?"contrast(1.16) brightness(.86) saturate(.86)":"contrast(1.12) saturate(.94)";
  ctx.globalCompositeOperation="source-over";
  for(let index=0;index<slices;index++) {
    const near=index/slices,far=(index+1)/slices;
    const yNear=near*near*height*1.85,yFar=far*far*height*1.85+1;
    const nearHalf=width*(.08+near*.88),farHalf=width*(.08+far*.88);
    const sourceY=(forward+index*sourceHeight)%image.naturalHeight;
    const safeHeight=Math.min(sourceHeight+2,image.naturalHeight-sourceY);
    if(safeHeight<=0)continue;
    ctx.save();
    // 멀리 있는 지표는 대기 산란으로 색과 명암이 약해지고 가까운 지표만 선명하게 보입니다.
    ctx.globalAlpha=baseAlpha*(.12+far*.88);
    ctx.beginPath();ctx.moveTo(-nearHalf,yNear);ctx.lineTo(nearHalf,yNear);
    ctx.lineTo(farHalf,yFar);ctx.lineTo(-farHalf,yFar);ctx.closePath();ctx.clip();
    const stripWidth=farHalf*2,shift=horizontalPhase*stripWidth;
    for(let tile=-1;tile<=1;tile++) {
      ctx.drawImage(image,0,sourceY,image.naturalWidth,safeHeight,-farHalf-shift+tile*stripWidth,yNear,stripWidth,yFar-yNear+2);
    }
    ctx.restore();
  }
  // 가까운 곳의 명암을 정리해 조종석 바로 앞에서 텍스처가 과도하게 밝아지지 않게 합니다.
  const shade=ctx.createLinearGradient(0,0,0,height*1.4);
  shade.addColorStop(0,"#08171b00");shade.addColorStop(.58,"#06131818");shade.addColorStop(1,"#02090db0");
  ctx.globalCompositeOperation="source-over";ctx.globalAlpha=1;ctx.fillStyle=shade;
  ctx.fillRect(-extent,0,extent*2,height*1.4);
  ctx.restore();
}

// 고해상도 텍스처 위에는 면을 덮는 도형 대신 월드 좌표에 고정된 작은 지표만 합성합니다.
// 이 레이어는 비행기 위치와 함께 투영되므로 배경과 따로 미끄러지지 않습니다.
function drawWorldDetailOverlay(theme) {
  const baseZ=Math.floor(lesson.z/900)*900;
  const altitudeFade=clamp(1-flight.altitude/6200,.24,.92);
  ctx.save();ctx.globalAlpha=altitudeFade;

  if(worldMap==="mountain-city"||worldMap==="night-city") {
    const road=theme.night?"#7da39a2d":"#263b3942";
    const shoulder=theme.night?"#101f2455":"#b8b79d20";
    // 큰 격자 대신 간격과 폭이 다른 실제 도로망처럼 드문 선형 지표를 사용합니다.
    for(const [index,x] of [-1080,-690,-380,420,760,1160].entries()) {
      const bend=(sceneryNoise(index*7.3)-.5)*95;
      runwayRectangle(x-5,baseZ-3800,x+5,baseZ+6100,shoulder);
      runwayRectangle(x-2.3,baseZ-3800,x+2.3,baseZ+6100,road);
      runwayRectangle(x+bend-1.2,baseZ-2100,x+bend+1.2,baseZ+3600,theme.night?"#b9d89025":"#d6d5b51c");
    }
    for(let row=-4;row<=6;row++) {
      const z=baseZ+row*900+(sceneryNoise(row*8.2)*180-90);
      const inset=120+sceneryNoise(row*3.9)*170;
      runwayRectangle(-1350,z,-inset,z+5.5,road);runwayRectangle(inset,z,1350,z+5.5,road);
      if(theme.night) for(let light=-1200;light<=1200;light+=145) {
        if(Math.abs(light)<inset)continue;
        drawProjectedMarker(light,z+2.8,"#efcf78",1.15);
      }
    }
    // 가까이 내려오면 활주로에서 충분히 떨어진 낮은 구조물만 더해 거대한 상자형 스카이라인을 피합니다.
    if(flight.altitude<950) drawLowAltitudeStructures(theme,baseZ);
  } else if(worldMap==="desert-base") {
    const track="#6c533c52",edge="#e0b87c1e";
    for(let lane=-3;lane<=3;lane++) {
      if(lane===0)continue;
      const x=lane*260+(sceneryNoise(lane*4.1)-.5)*75;
      runwayRectangle(x-4,baseZ-3300,x+4,baseZ+5600,track);
      runwayRectangle(x-1,baseZ-3300,x+1,baseZ+5600,edge);
    }
    for(let row=-3;row<=5;row++) {
      const z=baseZ+row*1080+sceneryNoise(row*5.7)*130;
      runwayRectangle(-1250,z,-180,z+4,track);runwayRectangle(180,z,1250,z+4,track);
    }
    if(flight.altitude<900) drawDesertBaseDetails(baseZ);
  } else if(worldMap==="ocean-islands") {
    // 고정된 잔물결과 작은 백파만 더해 수면의 축척을 보여주고 인공 섬 도형은 만들지 않습니다.
    for(let row=-5;row<=8;row++) {
      const z=baseZ+row*620,seed=row*12.7+baseZ*.001;
      for(let mark=0;mark<5;mark++) {
        const x=(sceneryNoise(seed+mark*4.8)-.5)*2500;
        const length=10+sceneryNoise(seed+mark*8.1)*34;
        runwayRectangle(x-length,z+mark*57,x+length,z+mark*57+1.1,"#d7f2ed38");
      }
    }
  }
  ctx.restore();
}

function drawLowAltitudeStructures(theme,baseZ) {
  const structures=[];
  for(let row=-3;row<=7;row++) for(const side of [-1,1]) {
    const seed=row*8.37+side*31.4;
    const z=baseZ+row*460+120+sceneryNoise(seed)*180;
    const x=side*(390+sceneryNoise(seed+3.2)*620);
    const camera=runwayCameraPoint(x,z);if(camera.z<90||camera.z>3200)continue;
    structures.push({
      camera,z,x,seed,
      width:28+sceneryNoise(seed+6.1)*58,
      height:8+sceneryNoise(seed+9.4)*(theme.night?34:22)
    });
  }
  structures.sort((a,b)=>b.camera.z-a.camera.z);
  const opacity=clamp(1-flight.altitude/950,.12,.58);
  structures.forEach(building=>drawCityBuilding(building,theme,opacity));
}

// 태양·달·수평선 안개를 더해 각 맵의 시간대와 대기감을 분명하게 만듭니다.
function drawSkyDetails(extent,theme,now,weather) {
  ctx.save();
  const night=theme.night&&weatherMode==="clear";
  const bodyX=weather.sunset?-width*.16:night?width*.18:-width*.31;
  const bodyY=weather.sunset?-height*.07:night?-height*.31:-height*.28;
  const radius=clamp(height*.055,24,54);
  if(!weather.overcast&&!weather.fog) {
    const glow=ctx.createRadialGradient(bodyX,bodyY,0,bodyX,bodyY,radius*(weather.sunset?4.8:3.3));
    glow.addColorStop(0,night?"#e9f4e7e8":weather.sunset?"#fff0b8fa":"#fff3b9f2");
    glow.addColorStop(.2,night?"#c9dce98a":weather.sunset?"#ff9b558f":"#ffd9859a");
    glow.addColorStop(1,"#ffffff00");
    ctx.fillStyle=glow;ctx.beginPath();ctx.arc(bodyX,bodyY,radius*(weather.sunset?4.8:3.3),0,Math.PI*2);ctx.fill();
    ctx.fillStyle=night?"#dce9e6":weather.sunset?"#fff1bd":"#fff1b5";ctx.beginPath();ctx.arc(bodyX,bodyY,radius,0,Math.PI*2);ctx.fill();
    if(night) {
      ctx.fillStyle="#162b3a";ctx.beginPath();ctx.arc(bodyX+radius*.34,bodyY-radius*.18,radius*.94,0,Math.PI*2);ctx.fill();
    }
  }
  const haze=ctx.createLinearGradient(0,-height*.18,0,height*.14);
  haze.addColorStop(0,"#d7eee000");haze.addColorStop(.55,night?"#5b839022":"#f1e8c526");haze.addColorStop(1,"#d7eee000");
  ctx.fillStyle=haze;ctx.fillRect(-extent,-height*.18,extent*2,height*.32);
  if(worldMap==="desert-base") {
    ctx.strokeStyle="#f4d49c22";ctx.lineWidth=1;
    for(let i=0;i<7;i++) {const y=-15-i*7-Math.sin(now*.00012+i)*2;ctx.beginPath();ctx.moveTo(-extent,y);ctx.lineTo(extent,y);ctx.stroke();}
  }
  ctx.restore();
}

// 월드 좌표의 지면 다각형을 카메라 앞에서 잘라 원근 투영합니다.
function drawGroundPolygon(worldPoints,fill,stroke=null) {
  const clipped=clipRunwayPolygon(worldPoints.map(([x,z])=>runwayCameraPoint(x,z)));
  if(clipped.length<3)return;
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  const screenPoints=clipped.map(point=>[point.x*focal/point.z,eyeHeight*focal/point.z]);
  path(screenPoints,fill,stroke);
}

function irregularAirfieldPolygon(inset=0) {
  const zValues=[-300,-80,320,760,1180,1600,2050,RUNWAY.length+320];
  const widths=[265,315,350,330,390,340,300,250].map(value=>Math.max(120,value-inset));
  const left=zValues.map((z,index)=>[-widths[index],z]);
  const right=zValues.map((z,index)=>[widths[index],z]).reverse();
  return left.concat(right);
}

// 활주로 바깥에도 이어지는 농지·도심 블록·사막 지표와 공항 부지를 만듭니다.
function drawRegionalGround(theme) {
  const baseZ=Math.floor(lesson.z/480)*480;
  const hasTexture=Boolean(terrainTextures[worldMap]?.complete&&terrainTextures[worldMap]?.naturalWidth);
  if(worldMap==="ocean-islands") {
    if(lesson.mode!=="free") {
      drawGroundPolygon(irregularAirfieldPolygon(0),"#d7c88a","#f0df9c88");
      drawGroundPolygon(irregularAirfieldPolygon(24),"#52755a","#8aa87988");
      drawGroundPolygon(irregularAirfieldPolygon(68),"#638363");
      drawAirportInfrastructure("#56675f","#899086");
    }
    return;
  }

  // 이미지가 준비되기 전의 대체 지형에서만 큰 색면을 사용합니다. 텍스처 위에 겹치면 모형 같은 사각 패턴이 생깁니다.
  if(!hasTexture) {
    const palettes=theme.desert
      ?["#b1875a88","#9c734e82","#c0996588","#7d593f82"]
      :theme.night?["#142b2d88","#10232688","#1b343388","#0b1d2288"]
        :["#58745c80","#6f86657d","#496b587d","#81906a78"];
    for(let row=-6;row<15;row++) for(let column=-5;column<=5;column++) {
      if(Math.abs(column)<1)continue;
      const seed=(baseZ/480+row)*17.1+column*4.7;
      const z=baseZ+row*480+8,x=column*310-145;
      const pad=10+sceneryNoise(seed)*18;
      runwayRectangle(x+pad,z+pad,x+285-pad,z+455-pad,palettes[Math.abs(Math.floor(seed))%palettes.length]);
      if(!theme.desert) {
        const edge=theme.night?"#48605c25":"#d3d9ac2d";
        runwayRectangle(x+pad,z+pad,x+pad+2,z+455-pad,edge);
        runwayRectangle(x+pad,z+pad,x+285-pad,z+pad+2,edge);
      }
    }
  }

  if(lesson.mode!=="free") {
    const outer=theme.desert?"#8c7359d8":theme.night?"#172b2ddd":"#5d765fd5";
    const inner=theme.desert?"#aa8b63e6":theme.night?"#203738e8":"#71866be2";
    drawGroundPolygon(irregularAirfieldPolygon(0),outer,theme.night?"#78948855":"#d2d9b144");
    drawGroundPolygon(irregularAirfieldPolygon(42),inner);
    drawAirportInfrastructure(theme.desert?"#64584d":"#485d58",theme.desert?"#9e9485":"#7e8c82");
  }

  if(!hasTexture&&worldMap==="mountain-city") drawRiverAndForest(baseZ,false);
  if(!hasTexture&&worldMap==="night-city") drawRiverAndForest(baseZ,true);
}

function drawAirportInfrastructure(taxiway,apron) {
  // 활주로와 평행한 유도로, 연결로, 주기장을 실제 지면 좌표에 배치합니다.
  runwayRectangle(-82,-60,-57,RUNWAY.length+90,taxiway);
  runwayRectangle(57,-60,82,RUNWAY.length+90,taxiway);
  for(const z of [170,620,1120,1660,2050]) {
    runwayRectangle(-82,z,-28,z+16,taxiway);runwayRectangle(28,z,82,z+16,taxiway);
  }
  runwayRectangle(-238,430,-88,870,apron);runwayRectangle(88,1260,235,1710,apron);
  runwayRectangle(-222,448,-104,854,"#26383b66");runwayRectangle(104,1278,219,1692,"#26383b66");
  for(let z=500;z<830;z+=82) drawGroundEllipse(-166,z,26,13,"#c4c89f3d");
  for(let z=1330;z<1680;z+=82) drawGroundEllipse(160,z,26,13,"#c4c89f3d");
}

function drawRiverAndForest(baseZ,night) {
  const left=[],right=[];
  for(let row=-7;row<=15;row++) {
    const z=baseZ+row*360;
    const center=-760+Math.sin(z*.0017)*150;
    left.push([center-45,z]);right.unshift([center+45,z]);
  }
  drawGroundPolygon(left.concat(right),night?"#0c2732bb":"#315f68aa",night?"#74aab733":"#a8d0c044");
  for(let row=-5;row<13;row++) {
    const z=baseZ+row*430+120;
    for(const side of [-1,1]) for(let tree=0;tree<5;tree++) {
      const seed=row*13+side*31+tree*2.9;
      const x=side*(420+sceneryNoise(seed)*520),tz=z+(sceneryNoise(seed+2)-.5)*250;
      drawProjectedTree(x,tz,night);
    }
  }
}

function drawProjectedTree(worldX,worldZ,night=false) {
  const camera=runwayCameraPoint(worldX,worldZ);if(camera.z<35||camera.z>3600)return;
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  const baseX=camera.x*focal/camera.z,baseY=eyeHeight*focal/camera.z;
  const treeHeight=7+sceneryNoise(worldX*.03+worldZ*.01)*10;
  const topY=(eyeHeight-treeHeight)*focal/camera.z,half=clamp(treeHeight*focal/camera.z*.32,.5,8);
  path([[baseX,topY],[baseX-half,baseY],[baseX+half,baseY]],night?"#071713cc":"#173d2fc4");
  ctx.fillStyle=night?"#38483a99":"#59432e99";ctx.fillRect(baseX-.5,baseY-2,1,2);
}

// 지면에 큰 색면을 흩뿌려 한 장의 그라데이션처럼 보이는 현상을 줄입니다.
function drawTerrainTexture(theme) {
  if(worldMap==="ocean-islands") return;
  const hasTexture=Boolean(terrainTextures[worldMap]?.complete&&terrainTextures[worldMap]?.naturalWidth);
  const baseZ=Math.floor(lesson.z/520)*520;
  if(!hasTexture) {
    const colors=theme.desert?["#c59a6555","#704b3650","#d2ae7550"]:theme.night?["#18353a55","#0a171d66","#25404944"]:["#79906b42","#304d454d","#8ba07338"];
    for(let row=-5;row<13;row++) {
      const index=baseZ/520+row,seed=index*8.17;
      for(const side of [-1,1]) {
        const z=baseZ+row*520+(sceneryNoise(seed+side*2.2)-.5)*240;
        const x=side*(260+sceneryNoise(seed+side*6.4)*760);
        drawGroundEllipse(x,z,130+sceneryNoise(seed+3.1)*260,60+sceneryNoise(seed+9.2)*150,colors[Math.abs(Math.floor(seed))%colors.length]);
      }
    }
  }
  if(!hasTexture&&theme.desert&&flight.altitude<1800) drawDesertBaseDetails(baseZ);
}

// 두 겹의 산맥은 서로 다른 속도로 흘러 가까운 능선과 먼 능선의 깊이를 만듭니다.
function drawMountainRanges(extent,theme) {
  for(const layer of [0,1,2]) {
    const step=[92,66,48][layer];
    const offset=flight.heading*[1.8,3.4,5.8][layer]+lesson.x*[.01,.022,.04][layer];
    const mountain=[[-extent,10]],ridge=[];
    for(let x=-extent;x<=extent;x+=step) {
      const sample=x+offset;
      const peak=[31,43,28][layer]*Math.abs(Math.sin(sample*[.0023,.0037,.0058][layer]+layer*.72))
        +[16,20,13][layer]*Math.abs(Math.sin(sample*[.0057,.0081,.012][layer]+.7));
      const y=-7-peak-[36,17,0][layer];mountain.push([x,y]);ridge.push([x,y]);
    }
    mountain.push([extent,10]);
    const fill=layer===0?theme.mountain[0]:layer===1?theme.mountain[1]:(theme.night?"#0b181dcc":theme.desert?"#704b3daa":"#304c49b8");
    path(mountain,fill);
    ctx.strokeStyle=theme.night?"#6a8a8a18":theme.desert?"#e0a97932":"#c7d8bf35";ctx.lineWidth=1;ctx.beginPath();
    ridge.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.stroke();
    // 산악 도시의 먼 높은 봉우리에는 얇은 설선을 표시합니다.
    if(worldMap==="mountain-city"&&layer===0) {
      ctx.strokeStyle="#dce7dc55";ctx.lineWidth=2;ctx.beginPath();
      ridge.forEach(([x,y],i)=>i?ctx.lineTo(x,y+Math.min(9,(Math.abs(Math.sin((x+offset)*.008))*8))):ctx.moveTo(x,y+5));ctx.stroke();
    }
  }
}

function drawNightSky(extent) {
  ctx.save();
  for(let i=0;i<65;i++) {
    const band=extent*2,x=((sceneryNoise(i*4.31)*band+flight.heading*8)%band)-extent;
    const y=-18-sceneryNoise(i*7.19)*height*.92;
    const size=sceneryNoise(i*2.07)>.88?1.6:.7;
    ctx.fillStyle=`rgba(220,239,225,${.25+sceneryNoise(i*9.7)*.6})`;ctx.fillRect(x,y,size,size);
  }
  // 낮은 별보다 밝은 몇 개의 항법 기준 별을 십자 광채로 표시합니다.
  ctx.strokeStyle="#dff6df88";ctx.lineWidth=.7;
  for(let i=0;i<8;i++) {
    const x=(sceneryNoise(i*18.2)*extent*2+flight.heading*5)% (extent*2)-extent;
    const y=-45-sceneryNoise(i*13.7)*height*.75,s=2+sceneryNoise(i)*2;
    ctx.beginPath();ctx.moveTo(x-s,y);ctx.lineTo(x+s,y);ctx.moveTo(x,y-s);ctx.lineTo(x,y+s);ctx.stroke();
  }
  ctx.restore();
}

function drawDistantSkyline(extent,theme) {
  // 높은 고도에서도 도시임을 알아볼 수 있도록 먼 수평선에 낮은 스카이라인을 둡니다.
  const step=24,offset=((flight.heading*7+lesson.x*.035)%step+step)%step;
  const skylineAlpha=clamp(1-flight.altitude/2300,0,.62);
  if(skylineAlpha<=0)return;
  ctx.save();ctx.globalAlpha=skylineAlpha;
  for(let x=-extent-offset,index=Math.floor((-extent-offset)/step);x<extent;x+=step,index++) {
    const seed=index*3.73,heightA=5+sceneryNoise(seed)*24;
    const buildingWidth=12+sceneryNoise(seed+2.4)*11,top=-heightA;
    ctx.fillStyle=theme.night?"#091820d9":"#314b4a80";ctx.fillRect(x,top,buildingWidth,heightA+2);
    if(sceneryNoise(seed+4.8)>.84) {
      ctx.fillStyle=theme.night?"#102a35e8":"#405b5780";ctx.fillRect(x+3,top-9,buildingWidth-6,10);
    }
    if(theme.night&&sceneryNoise(seed+8)>.38) {
      ctx.fillStyle=sceneryNoise(seed+9)>.5?"#e8dc75aa":"#8cc7b899";
      for(let wy=top+4;wy<-2;wy+=6) for(let wx=x+3;wx<x+buildingWidth-2;wx+=5) {
        if(sceneryNoise(seed+wx+wy)>.54)ctx.fillRect(wx,wy,1.2,1.2);
      }
    }
  }
  ctx.restore();
}

function drawCityScenery(theme) {
  // 고도에서는 항공 텍스처가 도시 규모를 표현하고, 개별 건물은 저고도에서만 드러납니다.
  const detailAlpha=clamp(1-(flight.altitude-250)/2500,.08,1);
  ctx.save();ctx.globalAlpha=detailAlpha;
  const roadBase=Math.floor(lesson.z/2000)*2000;
  // 활주로 양옆의 간선도로와 반복되는 연결도로. 중앙 160m는 비워 이착륙 시야를 보존합니다.
  runwayRectangle(-128,roadBase-5000,-117,roadBase+7000,theme.road);
  runwayRectangle(117,roadBase-5000,128,roadBase+7000,theme.road);
  for(let z=roadBase-4600;z<roadBase+6800;z+=520) {
    runwayRectangle(-720,z,-80,z+7,theme.crossRoad);
    runwayRectangle(80,z,720,z+7,theme.crossRoad);
    runwayRectangle(-720,z+3,-80,z+3.8,theme.night?"#b8dc9d28":"#d8d9bf22");
    runwayRectangle(80,z+3,720,z+3.8,theme.night?"#b8dc9d28":"#d8d9bf22");
  }
  drawCityTraffic(roadBase,theme);
  if(theme.night) drawRoadLights(roadBase);

  const centerRow=Math.floor(lesson.z/CITY_ROW_SPACING),buildings=[];
  for(let row=-9;row<=23;row++) {
    const index=centerRow+row;
    for(const side of [-1,1]) {
      const seed=index*2.17+side*19.3;
      const z=index*CITY_ROW_SPACING+(sceneryNoise(seed)-.5)*100;
      const x=side*(175+sceneryNoise(seed+2.3)*470);
      const camera=runwayCameraPoint(x,z);
      if(camera.z<70||camera.z>CITY_DRAW_DISTANCE) continue;
      const buildingHeight=theme.desert?8+sceneryNoise(seed+7.7)*20:18+sceneryNoise(seed+7.7)*92;
      buildings.push({camera,z,x,width:(theme.desert?55:32)+sceneryNoise(seed+4.1)*(theme.desert?85:62),height:buildingHeight,seed});
    }
  }
  // 먼 건물부터 그려 가까운 건물이 자연스럽게 앞을 가리게 합니다.
  buildings.sort((a,b)=>b.camera.z-a.camera.z);
  buildings.forEach(building=>drawCityBuilding(building,theme,detailAlpha));
  drawCityLandmarks(theme,roadBase);
  ctx.restore();
}

function drawCityTraffic(roadBase,theme) {
  // 도로를 따라 움직이는 작은 광점으로 도시가 정지된 모형처럼 보이지 않게 합니다.
  const colorA=theme.night?"#f7e27c":"#dce2bd99",colorB=theme.night?"#ef5f55":"#b8c8b488";
  const flow=(flight.distance*1500)%520;
  for(let row=-7;row<10;row++) {
    const z=roadBase+row*520+flow;
    for(const side of [-1,1]) {
      drawProjectedMarker(side*122,z,colorA,theme.night?2.3:1.25);
      drawProjectedMarker(side*126,z+115,colorB,theme.night?2.0:1.1);
    }
  }
}

function drawCityLandmarks(theme,roadBase) {
  if(theme.desert) return;
  // 서로 다른 높이의 통신탑 두 개가 도시 실루엣의 기준점 역할을 합니다.
  const towers=[{x:-410,z:roadBase+1480,h:145},{x:355,z:roadBase+2850,h:105}];
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  ctx.save();
  for(const tower of towers) {
    const camera=runwayCameraPoint(tower.x,tower.z);if(camera.z<70||camera.z>5000)continue;
    const x=camera.x*focal/camera.z,bottom=eyeHeight*focal/camera.z,top=(eyeHeight-tower.h)*focal/camera.z;
    ctx.strokeStyle=theme.night?"#8eaaa3aa":"#526862aa";ctx.lineWidth=clamp(900/camera.z,.7,2.2);
    ctx.beginPath();ctx.moveTo(x-8*focal/camera.z,bottom);ctx.lineTo(x,top);ctx.lineTo(x+8*focal/camera.z,bottom);ctx.stroke();
    for(let level=1;level<4;level++){const y=lerp(top,bottom,level/4),w=(y-top)/(bottom-top)*8*focal/camera.z;ctx.beginPath();ctx.moveTo(x-w,y);ctx.lineTo(x+w,y);ctx.stroke();}
    ctx.fillStyle=theme.night?"#ff554c":"#d6e2cf";ctx.shadowColor=ctx.fillStyle;ctx.shadowBlur=theme.night?8:2;ctx.beginPath();ctx.arc(x,top,clamp(430/camera.z,.8,2.4),0,Math.PI*2);ctx.fill();
  }
  ctx.restore();
}

function drawRoadLights(roadBase) {
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  ctx.save();ctx.fillStyle="#f4d776";ctx.shadowColor="#f4d776";ctx.shadowBlur=6;
  for(let z=roadBase-3800;z<roadBase+6200;z+=95) for(const x of [-123,123]) {
    const camera=runwayCameraPoint(x,z);if(camera.z<50||camera.z>3600) continue;
    const px=camera.x*focal/camera.z,py=eyeHeight*focal/camera.z;
    const radius=clamp(420/camera.z,.6,2.2);ctx.beginPath();ctx.arc(px,py,radius,0,Math.PI*2);ctx.fill();
  }
  ctx.restore();
}

function drawCityBuilding(building,theme,detailAlpha=1) {
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2,z=building.camera.z;
  const centerX=building.camera.x*focal/z,halfWidth=building.width*focal/z*.5;
  const bottom=eyeHeight*focal/z,top=(eyeHeight-building.height)*focal/z;
  if(centerX+halfWidth<-width||centerX-halfWidth>width||top>height*1.3) return;
  const haze=clamp(1-z/CITY_DRAW_DISTANCE,.16,.88);
  const shade=sceneryNoise(building.seed+11);
  ctx.save();ctx.globalAlpha=haze*detailAlpha;
  ctx.fillStyle=shade>.66?theme.buildings[0]:shade>.33?theme.buildings[1]:theme.buildings[2];
  ctx.fillRect(centerX-halfWidth,top,halfWidth*2,Math.max(1,bottom-top));
  // 햇빛을 받는 얇은 측면과 옥상으로 단순한 입체감을 냅니다.
  const roof=Math.min(7,900/z),slant=Math.min(9,1100/z);
  path([[centerX-halfWidth,top],[centerX+halfWidth,top],[centerX+halfWidth-slant,top-roof],[centerX-halfWidth+slant,top-roof]],theme.roof);
  ctx.fillStyle=theme.side;ctx.fillRect(centerX+halfWidth*.55,top,halfWidth*.45,Math.max(1,bottom-top));
  ctx.fillStyle=theme.night?"#9fc9bd2b":"#ecf3d31c";ctx.fillRect(centerX-halfWidth,top,Math.max(1,halfWidth*.08),Math.max(1,bottom-top));

  if(!theme.desert&&building.height>70&&bottom-top>18) {
    // 일부 고층 건물은 위쪽 폭을 줄인 단차형 실루엣을 사용합니다.
    const tierHeight=(bottom-top)*.32,tierInset=halfWidth*.17;
    ctx.fillStyle=shade>.5?theme.buildings[1]:theme.buildings[0];
    ctx.fillRect(centerX-halfWidth+tierInset,top-tierHeight*.2,halfWidth*2-tierInset*2,tierHeight);
  }

  // 높은 건물에는 옥상 설비와 안테나를 더해 반복되는 상자 모양을 깨 줍니다.
  if(!theme.desert&&building.height>62&&z<2500) {
    const antenna=clamp(1500/z,2,12);
    ctx.strokeStyle=theme.night?"#e87065aa":"#b9cbc288";ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(centerX,top-roof);ctx.lineTo(centerX,top-roof-antenna);ctx.stroke();
    if(theme.night&&sceneryNoise(building.seed+17)>.45) {ctx.fillStyle="#ff625a";ctx.globalAlpha=.9;ctx.fillRect(centerX-1,top-roof-antenna-1,2,2);}
  }

  if(z<1900&&bottom-top>9&&halfWidth>3) {
    ctx.fillStyle=theme.window;ctx.globalAlpha=haze*(theme.night ? .95 : .62);
    const columns=halfWidth>14?4:halfWidth>8?3:2,rows=clamp(Math.floor((bottom-top)/7),1,10);
    for(let row=0;row<rows;row++) for(let column=0;column<columns;column++) {
      if(sceneryNoise(building.seed+row*7+column*3)<.44) continue;
      const wx=centerX-halfWidth+(column+1)*(halfWidth*2)/(columns+1);
      const wy=top+(row+1)*(bottom-top)/(rows+1);
      ctx.fillRect(wx-1,wy-1,2,2);
    }
  }
  ctx.restore();
}

function drawDesertBaseDetails(baseZ) {
  // 군용 기지 느낌의 주기장, 격납고, 유도로를 활주로 양쪽에 배치합니다.
  for(let row=-3;row<8;row++) {
    const z=baseZ+row*700+180,side=row%2?-1:1;
    runwayRectangle(side*105,z,side*390,z+34,"#6d62556f");
    runwayRectangle(side*165,z+75,side*330,z+185,"#4c443c88");
    runwayRectangle(side*178,z+88,side*316,z+168,"#9c876d");
    runwayRectangle(side*190,z+97,side*304,z+157,"#725f4e");
    for(let pad=0;pad<3;pad++) drawGroundEllipse(side*(185+pad*60),z-35,22,15,"#b9a27a66");
    // 태양광 패널과 경계등을 기지 주변에 반복 배치합니다.
    for(let panel=0;panel<4;panel++) {
      const px=side*(430+panel*22),pz=z+60+(panel%2)*35;
      drawGroundEllipse(px,pz,14,7,"#213d496e");
      drawProjectedMarker(px,pz-12,"#efc978",1.35);
    }
  }
}

function drawGroundEllipse(centerX,centerZ,radiusX,radiusZ,color) {
  const cameraPoints=[];
  for(let i=0;i<22;i++) {
    const angle=Math.PI*2*i/22;
    cameraPoints.push(runwayCameraPoint(centerX+Math.cos(angle)*radiusX,centerZ+Math.sin(angle)*radiusZ));
  }
  const clipped=clipRunwayPolygon(cameraPoints);if(clipped.length<3)return;
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  path(clipped.map(point=>[point.x*focal/point.z,eyeHeight*focal/point.z]),color);
}

function drawOceanScenery() {
  const detailAlpha=clamp(1-(flight.altitude-200)/2100,0,1);
  ctx.save();ctx.globalAlpha=detailAlpha;
  const rowBase=Math.floor(lesson.z/700);
  // 훈련 활주로는 작은 공항 섬 위에 놓입니다.
  if(lesson.mode!=="free") runwayRectangle(-105,-180,105,RUNWAY.length+180,"#61765f");
  for(let row=-6;row<=12;row++) {
    const index=rowBase+row,seed=index*3.41;
    for(const side of [-1,1]) {
      const z=index*700+(sceneryNoise(seed+side*2.1)-.5)*230;
      const x=side*(150+sceneryNoise(seed+side*8.4)*780);
      const rx=65+sceneryNoise(seed+4.2)*150,rz=45+sceneryNoise(seed+9.6)*95;
      drawGroundEllipse(x,z,rx*1.14,rz*1.18,"#d2c98a");
      drawGroundEllipse(x,z,rx,rz,"#47745b");
      drawGroundEllipse(x-rx*.12,z-rz*.08,rx*.62,rz*.57,"#6f9566");
      // 큰 섬에는 작은 야자수 군집을 점으로 표현합니다.
      if(rx>130) for(let tree=0;tree<7;tree++) {
        const angle=tree*2.399+seed,dist=rx*(.15+sceneryNoise(seed+tree)*.48);
        drawProjectedMarker(x+Math.cos(angle)*dist,z+Math.sin(angle)*rz*.45,"#173f35",1.8);
      }
    }
  }
  const waveBase=Math.floor(lesson.z/420)*420;
  for(let z=waveBase-2500;z<waveBase+5200;z+=420) {
    const offset=(sceneryNoise(z*.01)-.5)*360;
    runwayRectangle(-900+offset,z,-260+offset,z+3,"#a9dae033");
    runwayRectangle(180-offset,z,760-offset,z+3,"#a9dae02b");
  }
  // 서로 다른 간격의 잔물결과 얕은 산호초가 수면의 크기와 방향을 읽게 합니다.
  for(let z=waveBase-1900;z<waveBase+4300;z+=145) {
    const seed=z*.017,offset=(sceneryNoise(seed)-.5)*950,length=55+sceneryNoise(seed+4)*170;
    runwayRectangle(offset-length,z,offset+length,z+1.2,"#d8f4ed20");
  }
  for(let reef=0;reef<5;reef++) {
    const seed=(rowBase+reef)*5.71;
    drawGroundEllipse((sceneryNoise(seed)-.5)*1350,rowBase*700+reef*520-900,70,25,"#55a6a425");
  }
  // 태양/달이 비치는 수면 중심부에 짧은 반사광을 겹칩니다.
  for(let i=0;i<18;i++) {
    const z=waveBase-300+i*170,widthAt=25+i*15;
    runwayRectangle(-widthAt,z,widthAt,z+1.5,worldMap==="night-city"?"#dcece633":"#d9f4df35");
  }
  ctx.restore();
}

function drawProjectedMarker(worldX,worldZ,color,size=2) {
  const camera=runwayCameraPoint(worldX,worldZ);if(camera.z<40||camera.z>CITY_DRAW_DISTANCE)return;
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2;
  const x=camera.x*focal/camera.z,y=eyeHeight*focal/camera.z,r=clamp(size*520/camera.z,.5,4);
  ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y-r,r,0,Math.PI*2);ctx.fill();
}

// 화면 중심에서 바깥으로 퍼지는 짧은 선으로 고속 공기 흐름을 표현합니다.
function drawWindStreaks(now) {
  const intensity=clamp((flight.speed-WIND_STREAK_START_SPEED)/110,0,1);
  if(intensity<=0||lesson.paused||lesson.result) return;
  const cx=width*.5,cy=height*.46,maxRadius=Math.hypot(width,height)*.58;
  ctx.save();ctx.lineCap="round";ctx.lineWidth=.7+intensity*1.15;
  for(let i=0;i<30;i++) {
    const angle=(i*2.399963+Math.sin(i*12.73)*.24)%(Math.PI*2);
    const phase=(now*.00022*(.55+flight.speed/150)+i*.173)%1;
    const radius=28+Math.pow(phase,1.7)*maxRadius;
    const length=(5+phase*46)*intensity;
    const cos=Math.cos(angle),sin=Math.sin(angle);
    ctx.strokeStyle=`rgba(219,238,228,${(.025+phase*.18)*intensity})`;
    ctx.beginPath();ctx.moveTo(cx+cos*radius,cy+sin*radius);ctx.lineTo(cx+cos*(radius+length),cy+sin*(radius+length));ctx.stroke();
  }
  ctx.restore();
}

function cockpitShake(now) {
  const highSpeed=clamp((flight.speed-105)/105,0,1);
  const groundRoll=(lesson.phase==="ground"||lesson.phase==="rollout")?clamp(flight.speed/85,0,1):0;
  const strength=highSpeed*1.7+groundRoll*1.1+(effects.stall?3.3:0);
  return {x:(Math.sin(now*.071)+Math.sin(now*.037)*.45)*strength,y:(Math.cos(now*.083)+Math.sin(now*.049)*.35)*strength*.62};
}

// 2D 원근 투영: 활주로의 평면 꼭짓점만 카메라 앞쪽으로 잘라 Canvas에 그립니다.
// 가까운 면을 자르지 않으면 활주로를 지나갈 때 화면을 가로지르는 거대 다각형이 생깁니다.
function runwayCameraPoint(x, z) {
  const yaw = radians(flight.heading - RUNWAY.heading);
  const dx = x - lesson.x, dz = z - lesson.z;
  return { x: dx * Math.cos(yaw) - dz * Math.sin(yaw), z: dx * Math.sin(yaw) + dz * Math.cos(yaw) };
}
function clipRunwayPolygon(points) {
  const output = [];
  for (let i = 0; i < points.length; i++) {
    const a = points[i], b = points[(i + 1) % points.length];
    const aInside = a.z >= RUNWAY_NEAR_CLIP, bInside = b.z >= RUNWAY_NEAR_CLIP;
    if (aInside) output.push(a);
    if (aInside !== bInside) {
      const t = (RUNWAY_NEAR_CLIP - a.z) / (b.z - a.z);
      output.push({ x: lerp(a.x, b.x, t), z: RUNWAY_NEAR_CLIP });
    }
  }
  return output;
}
function runwayRectangle(x1, z1, x2, z2, color) {
  const points = clipRunwayPolygon([[x1,z1],[x2,z1],[x2,z2],[x1,z2]].map(([x,z]) => runwayCameraPoint(x,z)));
  if (points.length < 3) return;
  const focal = height * 0.9;
  const eyeHeight = flight.altitude * FEET_TO_METERS + 2.2;
  path(points.map(p => [p.x * focal / p.z, eyeHeight * focal / p.z]), color);
}
function drawRunway() {
  const half = RUNWAY.width / 2, length = RUNWAY.length;
  runwayRectangle(-85,-100,85,length+100,"#67715c");
  runwayRectangle(-half-3,0,half+3,length,"#848877");
  runwayRectangle(-half,0,half,length,"#303b3d");
  runwayRectangle(-half+1,0,-half+1.6,length,"#d9e3d3");
  runwayRectangle(half-1.6,0,half-1,length,"#d9e3d3");
  for(let z=120;z<length-120;z+=65) runwayRectangle(-.6,z,.6,z+30,"#e1e9dc");
  for(let x=-23;x<=23;x+=6) {
    runwayRectangle(x,8,x+3,45,"#edf1dd");
    runwayRectangle(x,length-45,x+3,length-8,"#edf1dd");
  }
  // 접지 목표 지점의 두 흰 블록. 30/12 양방향에서 사용할 수 있습니다.
  for(const z of [290,length-320]) {
    runwayRectangle(-20,z,-12,z+35,"#e1e9dc");runwayRectangle(12,z,20,z+35,"#e1e9dc");
  }
  for(let z=0;z<=length;z+=70) {
    runwayRectangle(-half-2,z,-half-1,z+1.8,"#ffe9ac");runwayRectangle(half+1,z,half+2,z+1.8,"#ffe9ac");
  }
  runwayRectangle(-half,0,half,2,"#b7f6b7");runwayRectangle(-half,length-2,half,length,"#b7f6b7");
  if (lesson.phase === "airborne") {
    const forward = Math.cos(radians(flight.heading-RUNWAY.heading)) >= 0;
    const target = runwayCameraPoint(0, forward ? 300 : length-300);
    if (target.z > 100) {
      const x = target.x * height * .9 / target.z;
      const y = (flight.altitude*FEET_TO_METERS+2.2) * height * .9 / target.z;
      ctx.strokeStyle="#bcf7b0"; ctx.lineWidth=1; ctx.setLineDash([3,3]);
      ctx.beginPath(); ctx.arc(x,y,12,0,Math.PI*2); ctx.stroke();ctx.setLineDash([]);
      ctx.fillStyle="#d3f9bc";ctx.textAlign="center";ctx.font="10px Consolas,monospace";ctx.fillText("TOUCHDOWN",x,y-19);
    }
  }
  drawLandingAids();
}

function drawLandingAids() {
  const papi=papiGuidance();
  if(!papi||lesson.paused||lesson.result) return;
  const focal=height*.9,eyeHeight=flight.altitude*FEET_TO_METERS+2.2,half=RUNWAY.width/2;
  const side=papi.forward?-1:1;
  const lightZ=papi.targetZ;
  let labelX=0,labelY=0,visibleLights=0;

  // PAPI 네 개를 실제 활주로 옆 지상 좌표에 투영합니다.
  ctx.save();
  for(let i=0;i<4;i++) {
    const camera=runwayCameraPoint(side*(half+11+i*5),lightZ);
    if(camera.z<=RUNWAY_NEAR_CLIP) continue;
    const x=camera.x*focal/camera.z,y=eyeHeight*focal/camera.z;
    const white=i<papi.whiteCount,color=white?"#fffbe6":"#ff4f3e";
    const radius=clamp(2+420/camera.z,2.4,7);
    ctx.fillStyle=color;ctx.shadowColor=color;ctx.shadowBlur=8+radius;
    ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.fill();
    labelX+=x;labelY+=y;visibleLights++;
  }
  if(visibleLights&&papi.distance<1200) {
    ctx.shadowBlur=0;ctx.fillStyle=papi.whiteCount===2?"#bcf7b0":"#ffd18a";
    ctx.font="bold 9px Consolas,monospace";ctx.textAlign="center";
    ctx.fillText(`PAPI ${papi.whiteCount}W ${4-papi.whiteCount}R · ${papi.label}`,labelX/visibleLights,labelY/visibleLights-14);
  }
  ctx.restore();

  const prediction=predictTouchdown();
  if(!prediction) return;
  const camera=runwayCameraPoint(prediction.x,prediction.z);
  if(camera.z<=RUNWAY_NEAR_CLIP) return;
  const x=camera.x*focal/camera.z,y=eyeHeight*focal/camera.z;
  const color=prediction.safe?"#bcf7b0":"#ffd18a";
  ctx.save();ctx.translate(x,y);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;
  ctx.setLineDash([4,3]);ctx.beginPath();ctx.ellipse(0,0,17,8,0,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
  ctx.beginPath();ctx.moveTo(-23,0);ctx.lineTo(-8,0);ctx.moveTo(8,0);ctx.lineTo(23,0);ctx.moveTo(0,-13);ctx.lineTo(0,-5);ctx.moveTo(0,5);ctx.lineTo(0,13);ctx.stroke();
  if(camera.z<1200) {ctx.font="bold 9px Consolas,monospace";ctx.textAlign="center";ctx.fillText(`EST. ${prediction.status}`,0,-18);}
  ctx.restore();
}

function drawCheckpointRings() {
  const focal=height*.9;
  for (let index=mission.checkpoint;index<Math.min(MISSION_CHECKPOINTS.length,mission.checkpoint+2);index++) {
    const checkpoint=MISSION_CHECKPOINTS[index];
    const camera=runwayCameraPoint(checkpoint.x,checkpoint.z);
    if (camera.z<=RUNWAY_NEAR_CLIP) continue;
    const x=camera.x*focal/camera.z;
    const y=(flight.altitude-checkpoint.altitude)*FEET_TO_METERS*focal/camera.z;
    // 실제 원근 크기를 사용합니다. 가까워질 때 상한 115px에 붙어 있던 HUD 같은 움직임을 제거했습니다.
    const radius=clamp(CHECKPOINT_VISUAL_RADIUS*focal/camera.z,18,Math.hypot(width,height)*1.5);
    const current=index===mission.checkpoint;
    ctx.save();ctx.strokeStyle=current?"#ffd18a":"#bcf7b055";ctx.fillStyle=ctx.strokeStyle;
    ctx.lineWidth=current?clamp(radius*.018,3,11):1.5;ctx.setLineDash(current?[]:[6,6]);
    // 통과 직전에는 안쪽 잔상이 벌어져 전진 속도와 깊이를 강조합니다.
    if(current&&camera.z<360) {
      for(const scale of [.78,.9]) {
        ctx.globalAlpha=(1-camera.z/360)*.28;
        ctx.beginPath();ctx.arc(x,y,radius*scale,0,Math.PI*2);ctx.stroke();
      }
      ctx.globalAlpha=1;
    }
    ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
    const tick=clamp(radius*.12,10,45);
    ctx.beginPath();ctx.moveTo(x-radius-tick,y);ctx.lineTo(x-radius+tick,y);ctx.moveTo(x+radius-tick,y);ctx.lineTo(x+radius+tick,y);ctx.stroke();
    if(radius<height*.42) {
      ctx.font="bold 11px Consolas,monospace";ctx.textAlign="center";ctx.fillText(`GATE ${index+1} · ${checkpoint.altitude} FT`,x,y-radius-12);
    }
    ctx.restore();
  }
}

// 다음 링이 멀거나 화면 밖/뒤쪽에 있을 때도 회전 방향과 거리를 잃지 않게 합니다.
function checkpointScreenPosition(checkpoint) {
  const camera=runwayCameraPoint(checkpoint.x,checkpoint.z);
  if (camera.z<=RUNWAY_NEAR_CLIP) return null;
  const focal=height*.9;
  const localX=camera.x*focal/camera.z;
  const localY=(flight.altitude-checkpoint.altitude)*FEET_TO_METERS*focal/camera.z+flight.pitch*height*.012;
  const angle=-radians(flight.roll), cosine=Math.cos(angle), sine=Math.sin(angle);
  return {
    x:width/2+localX*cosine-localY*sine,
    y:height*.46+localX*sine+localY*cosine
  };
}

function drawCheckpointNavigator() {
  const checkpoint=MISSION_CHECKPOINTS[mission.checkpoint];
  if (!checkpoint || lesson.paused) return;
  const dx=checkpoint.x-lesson.x, dz=checkpoint.z-lesson.z;
  const distance=Math.hypot(dx,dz);
  const desiredHeading=(RUNWAY.heading+degrees(Math.atan2(dx,dz))+360)%360;
  const bearing=angleDifference(desiredHeading,flight.heading);
  const projected=checkpointScreenPosition(checkpoint);
  const comfortablyVisible=projected
    && projected.x>85 && projected.x<width-85
    && projected.y>70 && projected.y<height-105;
  // 가까운 링이 시야 안에 있으면 실제 링 자체만 보여줍니다.
  if (comfortablyVisible && distance<900) return;

  const side=bearing>8 ? 1 : bearing<-8 ? -1 : 0;
  const direction=side===0 ? "정면" : `${side>0?"우":"좌"} ${Math.abs(Math.round(bearing))}°`;
  const altitudeError=Math.round(checkpoint.altitude-flight.altitude);
  const altitudeText=Math.abs(altitudeError)<25 ? "고도 일치" : `${altitudeError>0?"↑":"↓"}${Math.abs(altitudeError)} FT`;
  const distanceText=distance>=1000 ? `${(distance/1000).toFixed(1)} KM` : `${Math.round(distance)} M`;
  const label=`${side>0?"▶":side<0?"◀":"◆"} GATE ${mission.checkpoint+1} · ${direction} · ${distanceText} · ${altitudeText}`;

  ctx.save();
  ctx.font="bold 11px Consolas,monospace";
  const boxWidth=Math.min(ctx.measureText(label).width+24,width-32);
  const x=clamp(width/2+side*width*.27,boxWidth/2+16,width-boxWidth/2-16);
  const y=Math.max(190,height*.28);
  ctx.fillStyle="#071620df";ctx.strokeStyle="#ffd18aaa";ctx.lineWidth=1;
  ctx.beginPath();ctx.roundRect(x-boxWidth/2,y-17,boxWidth,34,5);ctx.fill();ctx.stroke();
  ctx.fillStyle="#ffd18a";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillText(label,x,y);
  ctx.restore();
}

function drawGatePassEffect(now) {
  const effect=mission.gateEffect;
  if (!effect) return;
  const elapsed=now-effect.started;
  const duration=1700;
  if (elapsed>=duration) { mission.gateEffect=null; return; }
  const progress=clamp(elapsed/duration,0,1);
  const fade=1-progress;
  const centerX=width/2, centerY=height*.38;
  const success=effect.success!==false;
  const color=success?"188,247,176":"255,209,138";

  ctx.save();
  // 짧은 HUD 섬광과 바깥으로 퍼지는 원으로 통과 순간을 즉시 보여줍니다.
  ctx.fillStyle=`rgba(${color},${.12*fade})`;ctx.fillRect(0,0,width,height);
  ctx.translate(centerX,centerY);
  ctx.strokeStyle=`rgba(${color},${.9*fade})`;ctx.lineWidth=3;
  for(let ring=0;ring<2;ring++) {
    const radius=35+progress*150+ring*32;
    ctx.beginPath();ctx.arc(0,0,radius,0,Math.PI*2);ctx.stroke();
  }
  // 방사형 조각은 별도 이미지 없이 Canvas 선만 사용합니다.
  for(let i=0;i<16;i++) {
    const angle=Math.PI*2*i/16;
    const inner=55+progress*110, outer=inner+18+24*fade;
    ctx.beginPath();ctx.moveTo(Math.cos(angle)*inner,Math.sin(angle)*inner);
    ctx.lineTo(Math.cos(angle)*outer,Math.sin(angle)*outer);ctx.stroke();
  }
  const pop=Math.min(1,elapsed/180);
  ctx.globalAlpha=fade;
  ctx.fillStyle=success?"#bcf7b0":"#ffd18a";ctx.textAlign="center";ctx.textBaseline="middle";
  ctx.font=`bold ${Math.round(24+pop*10)}px Consolas,monospace`;
  ctx.fillText(success?(effect.final?"ALL GATES CLEAR":`GATE ${effect.gate} CLEAR`):`GATE ${effect.gate} MISSED`,0,-8);
  ctx.font="bold 12px Consolas,monospace";ctx.fillText(success?(effect.final?"LANDING APPROACH":"CHECKPOINT CONFIRMED"):"RETRY FROM LAST GATE",0,25);
  ctx.restore();
}

function drawPitchLadder() {
  ctx.strokeStyle = "#c5ffba66"; ctx.fillStyle = "#c5ffba99";
  ctx.lineWidth = 1; ctx.font = "11px Consolas, monospace"; ctx.textAlign = "center";
  for (let pitch = -30; pitch <= 30; pitch += 10) {
    const y = -pitch * height * 0.012;
    const length = pitch === 0 ? 95 : 55;
    ctx.setLineDash(pitch < 0 ? [5,5] : []);
    ctx.beginPath(); ctx.moveTo(-length, y); ctx.lineTo(-20, y); ctx.moveTo(20, y); ctx.lineTo(length, y); ctx.stroke();
    if (pitch) { ctx.fillText(String(Math.abs(pitch)), -length - 16, y + 4); ctx.fillText(String(Math.abs(pitch)), length + 16, y + 4); }
  }
  ctx.setLineDash([]);
}

// 훈련용 비행 유도점. 마름모를 중앙 자세 기준선에 맞추면 목표 경로에 가까워집니다.
function flightDirectorCommand() {
  if (lesson.mode === "free" || lesson.result) return null;
  if (lesson.phase === "ground" || lesson.phase === "rollout") {
    const headingError = angleDifference(RUNWAY.heading,flight.heading);
    return {
      x: clamp(-lesson.x/22 + headingError/16,-1,1), y: 0,
      label: lesson.phase === "ground" ? "CENTERLINE" : "ROLLOUT",
      onTarget: Math.abs(lesson.x)<3 && Math.abs(headingError)<4
    };
  }
  if (lesson.mode === "mission" && !mission.returning) {
    const checkpoint=MISSION_CHECKPOINTS[mission.checkpoint];
    const dx=checkpoint.x-lesson.x, dz=checkpoint.z-lesson.z;
    const desiredHeading=(RUNWAY.heading+degrees(Math.atan2(dx,dz))+360)%360;
    const headingError=angleDifference(desiredHeading,flight.heading);
    const altitudeError=flight.altitude-checkpoint.altitude;
    return {
      x:clamp(headingError/25,-1,1),
      y:clamp(altitudeError/300,-1,1),
      label:`GATE ${mission.checkpoint+1}`,
      onTarget:Math.abs(headingError)<5&&Math.abs(altitudeError)<70
    };
  }
  if (lesson.mode === "takeoff") {
    const targetPitch = lesson.takeoffNotified ? 4 : 8;
    return {
      x: clamp(-flight.roll/22,-1,1),
      y: clamp((flight.pitch-targetPitch)/12,-1,1),
      label: lesson.takeoffNotified ? "LEVEL" : "CLIMB",
      onTarget: Math.abs(flight.roll)<5 && Math.abs(flight.pitch-targetPitch)<2.5
    };
  }
  const guidance = getLandingGuidance();
  const targetPitch = 4;
  return {
    x: clamp(guidance.headingError/18,-1,1),
    y: guidance.flare ? clamp((flight.pitch-targetPitch)/8,-1,1) : clamp(guidance.altitudeError/220,-1,1),
    label: guidance.flare ? "FLARE" : "APPROACH",
    onTarget: Math.abs(guidance.headingError)<4
      && (guidance.flare ? Math.abs(flight.pitch-targetPitch)<2 : Math.abs(guidance.altitudeError)<60)
  };
}

function drawFlightDirector() {
  const command = flightDirectorCommand();
  if (!command || lesson.paused) return;
  const centerX=width/2, centerY=height*.46;
  const x=centerX+command.x*width*.18, y=centerY+command.y*height*.15;
  const color=command.onTarget?"#bcf7b0":"#ffd18a";
  ctx.save();ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=1.5;
  ctx.setLineDash([3,5]);ctx.beginPath();ctx.moveTo(centerX,centerY);ctx.lineTo(x,y);ctx.stroke();ctx.setLineDash([]);
  ctx.translate(x,y);ctx.rotate(Math.PI/4);ctx.strokeRect(-9,-9,18,18);ctx.rotate(-Math.PI/4);
  ctx.font="9px Consolas,monospace";ctx.textAlign="center";ctx.fillText(command.label,0,-17);
  ctx.restore();
}

function drawCockpit(now) {
  // 조종석과 자세 기준선은 카메라 기준으로 고정됩니다.
  const w = width, h = height;
  const shake=cockpitShake(now);ctx.save();ctx.translate(shake.x,shake.y);
  const shade = ctx.createLinearGradient(0,h*0.72,0,h);
  shade.addColorStop(0,"#14222b"); shade.addColorStop(1,"#060d13");
  const points = [[0,h*.77],[w*.15,h*.81],[w*.30,h*.94],[w*.38,h*.94],[w*.42,h*.84],[w*.58,h*.84],[w*.62,h*.94],[w*.70,h*.94],[w*.85,h*.81],[w,h*.77],[w,h],[0,h]];
  path(points, shade, "#64808055");
  path([[0,h*.78],[w*.15,h*.82],[w*.30,h*.95],[w*.26,h*.95],[w*.14,h*.85],[0,h*.81]],"#23353d");
  path([[w,h*.78],[w*.85,h*.82],[w*.70,h*.95],[w*.74,h*.95],[w*.86,h*.85],[w,h*.81]],"#23353d");
  ctx.strokeStyle = "#c2f6ab"; ctx.lineWidth = 2;
  const cx = w/2, cy = h*.46;
  ctx.beginPath(); ctx.moveTo(cx-46,cy);ctx.lineTo(cx-15,cy);ctx.lineTo(cx-9,cy+6);ctx.moveTo(cx+46,cy);ctx.lineTo(cx+15,cy);ctx.lineTo(cx+9,cy+6);ctx.stroke();
  ctx.beginPath();ctx.arc(cx,cy,3,0,Math.PI*2);ctx.stroke();
  // 기울기 표시 호는 고정이고 삼각 지시자만 비행 자세에 반응합니다.
  ctx.save();ctx.translate(cx,cy);ctx.strokeStyle="#c2f6ab77";ctx.lineWidth=1;
  const r = Math.min(w,h)*.21;
  ctx.beginPath();ctx.arc(0,0,r,-Math.PI*.76,-Math.PI*.24);ctx.stroke();
  for(let a=-45;a<=45;a+=15){const angle=radians(a-90);ctx.beginPath();ctx.moveTo(Math.cos(angle)*r,Math.sin(angle)*r);ctx.lineTo(Math.cos(angle)*(r+7),Math.sin(angle)*(r+7));ctx.stroke();}
  ctx.rotate(radians(flight.roll));path([[-4,-r+14],[4,-r+14],[0,-r+7]],"#c2f6ab");ctx.restore();
  ctx.restore();
}

function drawStick() {
  const cx = width * STICK_CENTER.x, cy = height * STICK_CENTER.y;
  const radius = Math.min(width,height) * STICK_VISUAL_RADIUS;
  const color = stick.grabbed ? "#bcf7b0" : "#8ba5a7";
  ctx.save();ctx.translate(cx,cy);
  // 큰 원은 손 인식 범위로 오해하기 쉬워 제거하고, 조종간 이동량만 짧은 눈금으로 표시합니다.
  ctx.strokeStyle="#9bc0ac25";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(-radius*.62,0);ctx.lineTo(radius*.62,0);ctx.moveTo(0,-radius*.42);ctx.lineTo(0,radius*.5);ctx.stroke();
  ctx.fillStyle="#0b151ddd";ctx.strokeStyle="#7c989977";ctx.beginPath();ctx.ellipse(0,radius*.58,27,12,0,0,Math.PI*2);ctx.fill();ctx.stroke();
  const applied=appliedStickInput();
  const x=applied.x*radius*.58, y=applied.y*radius*.42;
  ctx.lineCap="round";ctx.strokeStyle="#344b51";ctx.lineWidth=13;ctx.beginPath();ctx.moveTo(0,radius*.53);ctx.lineTo(x,y);ctx.stroke();
  ctx.lineWidth=2;ctx.strokeStyle=color;ctx.fillStyle=stick.grabbed?"#2d4b3c":"#1a2b34";
  ctx.beginPath();ctx.roundRect(x-17,y-22,34,47,10);ctx.fill();ctx.stroke();
  ctx.fillStyle=color;ctx.fillRect(x-7,y-12,14,3);ctx.fillStyle="#091217";ctx.fillRect(x-6,y+5,12,8);
  ctx.restore();
  // 사용자에게 카메라 좌표가 비행 화면의 어느 위치인지 알려줍니다.
  for (const [side, hand] of Object.entries(hands)) {
    if (!hand.detected || !hand.point) continue;
    const point = side === "Right" ? controlPoint(hand.point) : hand.point;
    const x = point.x * width, y = point.y * height;
    ctx.strokeStyle=side==="Right"?"#c4ffb8":"#91dcff";ctx.fillStyle=ctx.strokeStyle;ctx.lineWidth=1.5;
    ctx.beginPath();ctx.arc(x,y,hand.pinch?7:12,0,Math.PI*2);ctx.stroke();
    ctx.font="11px Consolas,monospace";ctx.textAlign="left";ctx.fillText(side==="Right"?"R":"L",x+17,y+4);
  }
}

function updateHud() {
  updateLessonHud();
  const easyControl = easyControlEnabled();
  $("vertical-pitch-assist").disabled = easyControl;
  $("pitch-negative-label").textContent = easyControl ? "손 아래로 · 하강" : "카메라 쪽 밀기 · 하강";
  $("pitch-positive-label").textContent = easyControl ? "손 위로 · 상승" : "몸 쪽 당김 · 상승";
  const depthReference = stick.grabbed ? stick.depthAnchor : calibration.palmScale;
  const depthValue = pitchControlInput(hands.Right);
  const depthPercent = Math.round(depthValue*100);
  $("depth-value").textContent = !hands.Right.detected ? "—" : (!easyControl && !depthReference) ? "PINCH" : `${depthPercent>0?"+":""}${depthPercent}%`;
  $("depth-meter").setAttribute("aria-valuenow",String(depthPercent));
  $("depth-marker").style.left = `${50+depthValue*50}%`;
  const rawDepthPercent = Math.round(depthPitchInput(hands.Right)*100);
  const rawVerticalPercent = Math.round(verticalPitchInput(hands.Right)*100);
  const relativeDistancePercent = Math.round(distanceChange(hands.Right)*100);
  if (easyControl) {
    $("distance-debug").textContent = "위로 이동 = 상승 · 아래로 이동 = 하강";
    const nearEdge = hands.Right.point && (hands.Right.point.y < HAND_EDGE_MARGIN || hands.Right.point.y > 1-HAND_EDGE_MARGIN);
    $("depth-status").textContent = !hands.Right.detected ? "한 손을 카메라에 보여주세요"
      : `자동 조종 활성 · 위아래 입력 ${rawVerticalPercent >= 0 ? "+" : ""}${rawVerticalPercent}%${nearEdge ? " · 화면 가장자리: 손을 중앙 쪽으로" : ""}`;
  } else {
    $("distance-debug").textContent = !depthReference ? "상대거리 — · 핀치하면 기준 자동 설정"
      : !depthReady(hands.Right, depthReference) ? "상대거리 측정 불가"
        : `상대거리 ${relativeDistancePercent >= 0 ? "+" : ""}${relativeDistancePercent}% · ${relativeDistancePercent > 0 ? "몸 쪽" : relativeDistancePercent < 0 ? "카메라 쪽" : "기준"}`;
    $("depth-status").textContent = !hands.Right.detected ? "오른손을 카메라에 보여주세요"
      : !depthReference ? "화면 어디서든 핀치하면 그 거리가 중립이 됩니다"
        : !depthReady(hands.Right, depthReference) ? ($("vertical-pitch-assist").checked ? `거리 감지 불안정 · 위아래 보조 ${rawVerticalPercent}%` : "거리 감지 불안정 · 손바닥을 정면으로 보여주세요")
          : `${lesson.paused ? "일시정지 · 입력하면 자동 시작" : stick.grabbed ? "조종 적용 중" : "핀치하면 적용"} · 거리 ${rawDepthPercent}%${$("vertical-pitch-assist").checked ? ` / 높이 ${rawVerticalPercent}%` : ""}${palmShapeStable(hands.Right) ? "" : " · 손바닥 정면 권장"}`;
  }
  ui["speed-value"].textContent = Math.round(flight.speed);
  ui["altitude-value"].textContent = Math.round(flight.altitude).toLocaleString("en-US");
  ui["throttle-value"].textContent = `${Math.round(flight.throttle)}%`;
  ui["pitch-value"].textContent = `${signed(flight.pitch)}°`;
  ui["roll-value"].textContent = `${signed(flight.roll)}°`;
  ui["elevator-value"].textContent = stick.grabbed ? `${signed(appliedStickInput().y*100,0)}%` : "0%";
  const radioAltitude = lesson.mode === "landing" && flight.altitude < 500 ? ` · RA ${Math.max(0,Math.round(flight.altitude))}` : "";
  ui["vertical-speed"].textContent = `V/S ${signed(flight.verticalSpeed * 60, 0)} FT/MIN${radioAltitude}`;
  ui["heading-value"].textContent = String(Math.round(flight.heading) % 360).padStart(3,"0");
  // 방위 눈금도 중심 방위에 맞춰 움직입니다.
  const headingLabels = document.querySelectorAll(".heading > span");
  [-30,-15,15,30].forEach((offset,i) => { headingLabels[i].textContent = String((Math.round(flight.heading)+offset+360)%360).padStart(3,"0"); });
  ui["throttle-fill"].style.height = `${flight.throttle}%`;
  ui["throttle-handle"].style.bottom = `${flight.throttle}%`;
  for (const side of ["Right","Left"]) {
    const key = side.toLowerCase();
    ui[`${key}-status`].textContent = hands[side].detected ? "DETECTED" : "NOT DETECTED";
    ui[`${key}-status`].classList.toggle("active",hands[side].detected);
    ui[`${key}-dot`].classList.toggle("off",!hands[side].detected);
  }
  ui["stick-status"].textContent=stick.grabbed?"GRABBED":"RELEASED";
  ui["stick-status"].classList.toggle("active",stick.grabbed);
  $("pinch-label").textContent = easyControl ? "AUTO:" : "PINCH:";
  ui["pinch-status"].textContent = easyControl ? (hands.Right.detected ? "ACTIVE" : "READY") : hands.Right.pinch ? "TRUE" : "FALSE";
  ui["pinch-status"].classList.toggle("active",easyControl ? hands.Right.detected : hands.Right.pinch);
  ui["hand-count"].textContent=`${Number(hands.Right.detected)+Number(hands.Left.detected)} / 2`;
  $("calibrate-button").disabled = !cameraActive || connectionStage !== "ready";
  $("calibrate-button").textContent = calibration.active ? "설정 취소 [C]" : "현재 자세로 설정 [C]";
  $("clear-calibration-button").disabled = !calibration.neutral && !calibration.active;
  $("calibration-status").textContent = calibration.note;
  $("calibration-progress").value = calibration.active ? calibration.held/CALIBRATION_HOLD_MS : calibration.neutral ? 1 : 0;
  ui["stick-hint"].textContent = connectionStage === "camera"
    ? "브라우저의 카메라 권한 요청에서 허용을 눌러주세요"
    : connectionStage === "model"
      ? "웹캠 연결됨 · 손 추적 모델을 준비하고 있습니다"
      : connectionStage === "model-error"
        ? "웹캠 영상은 연결됨 · 손 추적 오류 안내를 확인해주세요"
      : !cameraActive
        ? "먼저 오른쪽 위의 웹캠 연결을 눌러주세요"
        : !hands.Right.detected
          ? "카메라에 오른손을 보여주세요 · 인식되면 R 커서가 나타납니다"
          : stick.grabbed
            ? "핀치 유지 · 몸 쪽 당김 = 상승 · 카메라 쪽 밀기 = 하강"
            : "오른손 위치와 관계없이 엄지와 검지를 붙이면 조종간을 잡습니다";
  if (easyControl && hands.Right.detected) {
    ui["stick-hint"].textContent = "이지 조종 활성 · 오른손 좌우 = Roll · 위/아래 = 상승/하강";
  }
  if (stick.grabbed && lesson.mode === "takeoff" && lesson.phase === "ground") {
    const elevator = signed(stick.y*100,0);
    ui["stick-hint"].textContent = flight.speed < RUNWAY.rotationSpeed
      ? `ELEVATOR ${elevator}% 인식 · ${RUNWAY.rotationSpeed} KTS 전에는 PITCH 0°로 지상 활주합니다`
      : easyControl
        ? `ELEVATOR ${elevator}% · 손을 위로 올리면 지금 기수가 올라갑니다`
        : `ELEVATOR ${elevator}% · 몸 쪽으로 당기면 지금 기수가 올라갑니다`;
  }
  if (calibration.active) ui["stick-hint"].textContent = "중심 설정 중 · 오른손을 편한 위치에 유지하세요";
}

function updateLessonHud() {
  const training = lesson.mode !== "free";
  $("lesson-hud").hidden = !training;
  $("pause-notice").hidden = !(lesson.paused || calibration.active) || Boolean(lesson.result);
  $("pause-notice").textContent = calibration.active ? "중심 설정 중 · 비행 일시정지" : "일시정지 · 비행 시작을 누르거나 조종간을 움직이세요";
  $("pause-button").textContent = lesson.paused ? "비행 시작" : "일시정지";
  $("pause-button").disabled = Boolean(lesson.result);
  $("brake-button").disabled = !training || Boolean(lesson.result) || lesson.phase === "airborne";
  $("brake-button").textContent = lesson.brake ? "브레이크 ON" : "브레이크 OFF";
  $("brake-button").setAttribute("aria-pressed", String(lesson.brake));
  for (const [id, mode] of [["free-flight-button","free"],["takeoff-button","takeoff"],["approach-button","landing"],["mission-button","mission"]]) $(id).setAttribute("aria-pressed",String(lesson.mode===mode));
  $("mode-label").textContent = lesson.mode === "free" ? "FREE FLIGHT" : lesson.mode === "takeoff" ? "TAKEOFF TRAINING" : lesson.mode === "landing" ? "LANDING TRAINING" : "FULL MISSION";
  $("header-mode").textContent = $("mode-label").textContent;
  if (!training) return;
  const director = flightDirectorCommand();
  $("director-status").textContent = director?.onTarget ? "정렬" : "마름모 추적";
  $("lesson-route").textContent = lesson.mode === "mission"
    ? `${mission.returning ? "RETURN" : `GATE ${Math.min(mission.checkpoint+1,MISSION_CHECKPOINTS.length)}/${MISSION_CHECKPOINTS.length}`} · ${formatMissionTime(mission.elapsed)}${mission.retries?` · RETRY ${mission.retries}`:""}`
    : "RWY 30 / 12";
  $("distance-label").textContent="활주로";$("lateral-label").textContent="중심선";$("altitude-label").textContent="진입 높이";
  if (lesson.mode === "mission" && lesson.phase === "airborne" && !mission.returning) {
    const checkpoint=MISSION_CHECKPOINTS[mission.checkpoint];
    const horizontal=Math.hypot(checkpoint.x-lesson.x,checkpoint.z-lesson.z);
    const altitudeError=flight.altitude-checkpoint.altitude;
    const desiredHeading=(RUNWAY.heading+degrees(Math.atan2(checkpoint.x-lesson.x,checkpoint.z-lesson.z))+360)%360;
    const bearing=angleDifference(desiredHeading,flight.heading);
    $("distance-label").textContent="다음";$("lateral-label").textContent="거리";$("altitude-label").textContent="목표 고도";
    $("runway-distance").textContent=`GATE ${mission.checkpoint+1}/${MISSION_CHECKPOINTS.length}`;
    $("lateral-error").textContent=`${Math.round(horizontal)} M`;
    $("glide-error").textContent=`${altitudeError>=0?"높음":"낮음"} ${Math.abs(Math.round(altitudeError))} FT`;
    const profile=MISSION_PROFILES[mission.profileId];
    $("target-speed").textContent=profile?`${profile.speed[0]}–${profile.speed[1]} KTS`:"110–150 KTS";
    $("director-status").textContent = director?.onTarget ? "정렬" : Math.abs(bearing)<8 ? "정면" : `${bearing>0?"우":"좌"} ${Math.abs(Math.round(bearing))}°`;
    $("phase-label").textContent=`체크포인트 ${mission.checkpoint+1} 접근`;
    $("lesson-instruction").textContent=mission.gateStatus
      ? mission.gateStatus
      : `GATE ${mission.checkpoint+1} · ${checkpoint.altitude} FT · 링 중심과 유도 마름모를 맞추세요.`;
    return;
  }
  const forward = Math.cos(radians(flight.heading-RUNWAY.heading)) >= 0;
  const thresholdDistance = forward ? -lesson.z : lesson.z-RUNWAY.length;
  const targetDistance = forward ? 300-lesson.z : lesson.z-(RUNWAY.length-300);
  const desiredAltitude = Math.max(0,targetDistance) * Math.tan(radians(3)) / FEET_TO_METERS;
  const glideError = flight.altitude-desiredAltitude;
  const landing = lesson.mode === "landing" || (lesson.mode === "mission" && mission.returning) ? getLandingGuidance() : null;
  $("runway-distance").textContent = thresholdDistance > 0 ? `${(thresholdDistance/1000).toFixed(2)} KM` : onRunway(lesson.x,lesson.z) ? `${Math.round(forward ? RUNWAY.length-lesson.z : lesson.z)} M 남음` : "활주로 밖";
  $("lateral-error").textContent = Math.abs(lesson.x)<2 ? "중앙" : `${lesson.x>0?"우":"좌"} ${Math.abs(lesson.x).toFixed(0)} M`;
  $("glide-error").textContent = lesson.phase !== "airborne" ? "—" : `${glideError >= 0 ? "높음" : "낮음"} ${Math.abs(glideError).toFixed(0)} FT`;
  $("target-speed").textContent = landing ? `${landing.targetSpeed} KTS` : lesson.phase === "ground" ? "70+ KTS" : "상승 유지";
  if(landing&&lesson.phase==="airborne") {
    const papi=papiGuidance(),prediction=predictTouchdown();
    $("director-status").textContent=`PAPI ${papi.whiteCount}W/${4-papi.whiteCount}R${prediction?` · ${prediction.status}`:""}`;
  }
  const phases = {ground:"지상 활주",airborne:lesson.takeoffNotified?"이륙 완료 · 비행 중":lesson.tookOff?"이륙 상승":"접근 / 비행",rollout:"접지 완료 · 제동",complete:"착륙 성공",failed:"훈련 종료"};
  $("phase-label").textContent = lesson.paused ? "준비 · 비행 일시정지" : landing && lesson.phase === "airborne" ? landing.stage : phases[lesson.phase];
  let hint;
  if (lesson.result) hint = lesson.result.success ? "활주로 안에 정지했습니다." : "결과와 재시도 안내를 확인하세요.";
  else if (lesson.paused) hint = "‘비행 시작’을 누르거나 조종간을 잡고 움직이면 자동으로 시작합니다. P 키도 사용할 수 있습니다.";
  else if (lesson.phase === "ground") hint = lesson.brake ? "브레이크를 해제하세요. 스로틀 80% 이상으로 가속합니다."
    : flight.speed < RUNWAY.rotationSpeed ? "스로틀 80% 이상 · 중심선 유지 · 70 KTS까지 가속하세요."
      : flight.pitch < RUNWAY.rotationPitch ? `이륙 속도 도달 · INPUT을 +로 유지해 PITCH를 +4° 이상 올리세요. 현재 ${signed(flight.pitch)}°`
        : "기수 상승 완료 · 이륙 중입니다.";
  else if (lesson.phase === "rollout") hint = "자동 제동 중 · 날개를 수평으로 하고 활주로 중심선을 유지하세요.";
  else if (lesson.mode === "takeoff") hint = lesson.takeoffNotified && lesson.z > RUNWAY.length
    ? "이륙 완료! 선회해 돌아오거나 ‘착륙 연습’을 눌러 접근 위치로 이동하세요."
    : "노란 CLIMB 마름모를 중앙에 맞춰 고도 100 FT까지 안정적으로 상승하세요.";
  else if (lesson.mode === "mission" && !mission.returning) hint = "이륙 후 GATE 1 유도 마름모를 따라 상승하세요.";
  else if (landing?.flare) hint = easyControlEnabled()
    ? `FLARE · 목표 ${landing.targetSpeed} KTS · 오른손을 조금 올려 PITCH +3~5°로 접지하세요.`
    : `FLARE · 목표 ${landing.targetSpeed} KTS · 조종간을 조금 당겨 PITCH +3~5°로 접지하세요.`;
  else if (-flight.verticalSpeed > RUNWAY.maxSink) hint = "하강이 빠릅니다 · 유도 마름모를 따라 하강률을 줄이세요.";
  else if (flight.speed > landing.targetSpeed+10) hint = `접근 속도가 높습니다 · 출력을 줄여 ${landing.targetSpeed} KTS에 맞추세요.`;
  else if (flight.speed < landing.targetSpeed-12) hint = `속도가 낮습니다 · 출력을 올려 ${landing.targetSpeed} KTS에 맞추세요.`;
  else if (Math.abs(landing.headingError)>6) hint = "활주로 방향이 어긋났습니다 · 노란 마름모 방향으로 완만하게 Roll 하세요.";
  else if (Math.abs(landing.altitudeError)>80) hint = landing.altitudeError>0 ? "진입 경로보다 높습니다 · 마름모를 따라 조금 내려가세요." : "진입 경로보다 낮습니다 · 마름모를 따라 조금 올라가세요.";
  else hint = `안정 접근 · ${landing.targetSpeed} KTS · 마름모를 중앙에 유지하세요.`;
  $("lesson-instruction").textContent = hint;
}

function animate(now) {
  const dt = lastFrame ? Math.min((now-lastFrame)/1000,MAX_DELTA_TIME) : 1/60;
  lastFrame=now;
  trackHands(now);
  updateFlight(dt,now);
  updateFlightEffects(now);
  syncRealEarthCamera();
  drawWorld(now);drawCockpit(now);drawFlightDirector();drawStick();
  updateEngineSound();
  if(now-lastHud>100){updateHud();lastHud=now;}
  requestAnimationFrame(animate);
}

function setMapPanel(open) {
  if(open) setWeatherPanel(false,false);
  if(open) setMissionPanel(false,false);
  if(open) setEarthPanel(false,false);
  $("map-panel").hidden=!open;
  $("map-button").setAttribute("aria-expanded",String(open));
  if(open) $("map-panel").querySelector(`[data-map="${worldMap}"]`)?.focus();
  else $("map-button").focus({preventScroll:true});
}

function setEarthPanel(open,restoreFocus=true) {
  if(open) {
    setWeatherPanel(false,false);setMissionPanel(false,false);
    if(!$('map-panel').hidden){$('map-panel').hidden=true;$('map-button').setAttribute('aria-expanded','false');}
    if(realEarth.enabled)setEarthStatus(`연결됨 · ${EARTH_LOCATIONS[worldMap].label} 무료 3D 지도`,"active");
  }
  $("earth-panel").hidden=!open;$("earth-button").setAttribute("aria-expanded",String(open));
  if(open)$(realEarth.enabled?"earth-disable-button":"earth-enable-button").focus();else if(restoreFocus)$("earth-button").focus({preventScroll:true});
}

function setWeatherPanel(open,restoreFocus=true) {
  if(open) setMissionPanel(false,false);
  if(open) setEarthPanel(false,false);
  if(open&&$("map-panel")&&!$("map-panel").hidden) {
    $("map-panel").hidden=true;$("map-button").setAttribute("aria-expanded","false");
  }
  $("weather-panel").hidden=!open;
  $("weather-button").setAttribute("aria-expanded",String(open));
  if(open) $("weather-panel").querySelector(`[data-weather="${weatherMode}"]`)?.focus();
  else if(restoreFocus) $("weather-button").focus({preventScroll:true});
}

function missionBest(profileId) {
  try{return Number(localStorage.getItem(`aeronaut-mission-best-${profileId}`))||0;}catch{return 0;}
}

function saveMissionBest(profileId,score) {
  const best=Math.max(missionBest(profileId),score);
  try{localStorage.setItem(`aeronaut-mission-best-${profileId}`,String(best));}catch{}
  refreshMissionBests();
}

function refreshMissionBests() {
  document.querySelectorAll("[data-best]").forEach(label=>{
    const best=missionBest(label.dataset.best);label.textContent=best?`BEST ${best}점`:"BEST —";
  });
}

function setMissionPanel(open,restoreFocus=true) {
  if(open) {
    if(!$("map-panel").hidden){$("map-panel").hidden=true;$("map-button").setAttribute("aria-expanded","false");}
    if(!$("weather-panel").hidden){$("weather-panel").hidden=true;$("weather-button").setAttribute("aria-expanded","false");}
    if(!$("earth-panel").hidden){$("earth-panel").hidden=true;$("earth-button").setAttribute("aria-expanded","false");}
    selectMissionCard(selectedMissionId);
    refreshMissionBests();
  }
  $("mission-panel").hidden=!open;
  $("mission-button").setAttribute("aria-expanded",String(open));
  if(open) $("mission-panel").querySelector(`[data-mission="${selectedMissionId}"]`)?.focus();
  else if(restoreFocus) $("mission-button").focus({preventScroll:true});
}

function selectMissionCard(profileId) {
  const profile=MISSION_PROFILES[profileId];if(!profile)return;
  selectedMissionId=profileId;
  try{localStorage.setItem("aeronaut-mission",profileId);}catch{}
  document.querySelectorAll("[data-mission]").forEach(button=>button.setAttribute("aria-pressed",String(button.dataset.mission===profileId)));
  $("mission-brief-title").textContent=profile.title;
  $("mission-difficulty").textContent=`난이도 ${profile.difficulty} · ${profile.time} · ${profile.speed[0]}–${profile.speed[1]} KTS`;
  $("mission-description").textContent=profile.description;
  $("mission-weather").textContent=`현재 날씨 · ${WEATHER_PRESETS[weatherMode].label}`;
}

function beginSelectedMission() {
  const profile=MISSION_PROFILES[selectedMissionId];if(!profile)return;
  mission.profileId=selectedMissionId;
  MISSION_CHECKPOINTS=profile.checkpoints.map(point=>({...point}));
  selectWorldMap(profile.map,false);
  setMissionPanel(false,false);
  startLesson("mission");
  showMessage(`${profile.title} · ${WEATHER_PRESETS[weatherMode].label} · 이륙 후 GATE 1로 향하세요.`,false,6500);
  $("pause-button").focus({preventScroll:true});
}

function selectWorldMap(map,notify=true) {
  if(!MAP_THEMES[map]) return;
  worldMap=map;
  try{localStorage.setItem("aeronaut-map",map);}catch{}
  $("flight").dataset.map=map;
  $("map-button").textContent=`맵 · ${MAP_THEMES[map].label}`;
  document.querySelectorAll("[data-map]").forEach(button=>button.setAttribute("aria-pressed",String(button.dataset.map===map)));
  if(realEarth.enabled){syncRealEarthCamera(true);setEarthStatus(`연결됨 · ${EARTH_LOCATIONS[map].label} 무료 3D 지도`,"active");}
  if(notify) {
    setMapPanel(false);
    showMessage(realEarth.enabled?`${EARTH_LOCATIONS[map].label} 실제 지역으로 이동했습니다.`:`${MAP_THEMES[map].label} 맵으로 변경했습니다.`,false,3500);
  }
}

function selectWeather(mode,notify=true) {
  if(!WEATHER_PRESETS[mode])return;
  weatherMode=mode;
  try{localStorage.setItem("aeronaut-weather",mode);}catch{}
  $("flight").dataset.weather=mode;
  $("weather-button").textContent=`날씨 · ${WEATHER_PRESETS[mode].label}`;
  document.querySelectorAll("[data-weather]").forEach(button=>button.setAttribute("aria-pressed",String(button.dataset.weather===mode)));
  if($("mission-weather")) $("mission-weather").textContent=`현재 날씨 · ${WEATHER_PRESETS[mode].label}`;
  if(notify) {
    setWeatherPanel(false);
    const wind=WEATHER_PRESETS[mode].wind>=.4?" · 강한 돌풍":WEATHER_PRESETS[mode].wind>=.2?" · 약한 돌풍":"";
    showMessage(`${WEATHER_PRESETS[mode].label} 날씨로 변경했습니다${wind}.`,false,4200);
  }
}

$("camera-button").addEventListener("click", () => {
  if(cameraActive){stopCamera();showMessage("웹캠 연결을 종료했습니다. 현재 스로틀은 유지됩니다.",false,4000);}
  else startCamera();
});
$("reset-button").addEventListener("click", () => {
  startLesson(lesson.mode);
});
$("free-flight-button").addEventListener("click", () => startLesson("free"));
$("takeoff-button").addEventListener("click", () => startLesson("takeoff"));
$("approach-button").addEventListener("click", () => startLesson("landing"));
$("mission-button").addEventListener("click", () => setMissionPanel($("mission-panel").hidden));
$("mission-close-button").addEventListener("click",()=>setMissionPanel(false));
document.querySelectorAll("[data-mission]").forEach(button=>button.addEventListener("click",()=>selectMissionCard(button.dataset.mission)));
$("mission-start-button").addEventListener("click",beginSelectedMission);
$("retry-button").addEventListener("click", () => { startLesson(lesson.mode); $("pause-button").focus(); });
function togglePause() { if (!lesson.result) { lesson.paused = !lesson.paused; updateHud(); } }
function toggleBrake() { if (lesson.mode !== "free" && !lesson.result && lesson.phase !== "airborne") { lesson.brake = !lesson.brake; updateHud(); } }
$("pause-button").addEventListener("click", togglePause);
$("brake-button").addEventListener("click", toggleBrake);
$("audio-button").addEventListener("click",() => setSoundMuted(!sound.muted));
$("map-button").addEventListener("click",()=>setMapPanel($("map-panel").hidden));
$("map-close-button").addEventListener("click",()=>setMapPanel(false));
document.querySelectorAll("[data-map]").forEach(button=>button.addEventListener("click",()=>selectWorldMap(button.dataset.map)));
$("earth-button").addEventListener("click",()=>setEarthPanel($("earth-panel").hidden));
$("earth-close-button").addEventListener("click",()=>setEarthPanel(false));
$("earth-enable-button").addEventListener("click",enableRealEarth);
$("earth-disable-button").addEventListener("click",()=>disableRealEarth());
$("weather-button").addEventListener("click",()=>setWeatherPanel($("weather-panel").hidden));
$("weather-close-button").addEventListener("click",()=>setWeatherPanel(false));
document.querySelectorAll("[data-weather]").forEach(button=>button.addEventListener("click",()=>selectWeather(button.dataset.weather)));
document.addEventListener("pointerdown",ensureAudio,{once:true});
document.addEventListener("keydown", event => {
  if(event.code==="Escape"&&!$("earth-panel").hidden){event.preventDefault();setEarthPanel(false);return;}
  if(event.code==="Escape"&&!$("map-panel").hidden){event.preventDefault();setMapPanel(false);return;}
  if(event.code==="Escape"&&!$("weather-panel").hidden){event.preventDefault();setWeatherPanel(false);return;}
  if(event.code==="Escape"&&!$("mission-panel").hidden){event.preventDefault();setMissionPanel(false);return;}
  if (event.repeat || event.isComposing || event.ctrlKey || event.altKey || event.metaKey || /^(INPUT|SELECT|TEXTAREA)$/.test(event.target.tagName) || event.target.isContentEditable) return;
  if (event.code === "KeyP") { event.preventDefault(); togglePause(); }
  if (event.code === "KeyB") { event.preventDefault(); toggleBrake(); }
  if (event.code === "KeyC") { event.preventDefault(); toggleCalibration(); }
});
function toggleCalibration() {
  if (calibration.active) cancelCalibration();
  else if (!cameraActive || connectionStage !== "ready") showMessage("먼저 웹캠을 연결하고 손 추적 준비가 끝난 뒤 C 키를 눌러주세요.", false, 5000);
  else startCalibration();
  updateHud();
}
$("calibrate-button").addEventListener("click", toggleCalibration);
$("clear-calibration-button").addEventListener("click", () => { clearCalibration(); updateHud(); });
$("control-mode").addEventListener("change", () => {
  releaseStick();
  stick.x = 0; stick.y = 0;
  showMessage(easyControlEnabled()
    ? "이지 조종: 손을 보여주고 좌우·위아래로 움직이세요. 핀치는 필요 없습니다."
    : "고급 조종: 어디서든 핀치한 뒤 좌우로 움직이고 앞뒤로 당기거나 미세요.", false, 6000);
  updateHud();
});
$("control-response").addEventListener("change", event => {
  const labels={smooth:"부드럽게",normal:"보통",fast:"빠르게"};
  showMessage(`조종 반응: ${labels[event.target.value]} · 손 떨림 보정은 계속 적용됩니다.`,false,4000);
  updateHud();
});
$("vertical-pitch-assist").addEventListener("change", () => { stick.y=0; updateHud(); });
$("swap-hands").addEventListener("change",() => { clearHands(); clearCalibration(); });
$("camera-select").addEventListener("change", () => {
  if (connectionStage === "camera" || connectionStage === "model") return;
  // 선택만 바꾸면 권한 요청을 하지 않습니다. 이미 연결 중일 때에만 즉시 전환합니다.
  if (cameraActive) { stopCamera(); startCamera(); }
});
navigator.mediaDevices?.addEventListener("devicechange", refreshCameraList);
refreshCameraList();
document.addEventListener("visibilitychange",() => {
  clearHands(); lastFrame=0;
  if (document.hidden && lesson.mode !== "free" && !lesson.result) lesson.paused = true;
});
window.addEventListener("pagehide",() => {stopCamera();landmarker?.close();landmarker=null;sound.context?.close();destroyEarthViewer();if("speechSynthesis" in window)window.speechSynthesis.cancel();});
new ResizeObserver(resizeCanvas).observe(canvas);
selectWorldMap(worldMap,false);selectWeather(weatherMode,false);selectMissionCard(selectedMissionId);refreshMissionBests();resizeCanvas();updateHud();requestAnimationFrame(animate);
if(realEarth.restore)enableRealEarth();
if (window.location.protocol === "file:") {
  $("local-server-link").hidden = false;
  showMessage("파일을 직접 열었습니다. 아래 버튼으로 로컬 서버에서 열거나 VS Code Live Server를 사용해주세요.", true);
}
