const { spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');
const os = require('os');

const isWindows = process.platform === 'win32';
const exeName = isWindows ? 'ninja.exe' : 'ninja';

// xpm создаёт в xpacks/.bin обёртки (ninja.cmd, arm-none-eabi-gcc без
// расширения), которые CMake/Ninja запустить не могут. Поэтому ищем
// настоящие бинарники из распакованных пакетов и подсовываем их первыми в PATH.
function xpackRoots() {
  const roots = [path.resolve('xpacks')];
  if (isWindows) {
    if (process.env.APPDATA) roots.push(path.join(process.env.APPDATA, 'xPacks'));
  } else {
    roots.push(
      path.join(os.homedir(), '.local', 'xPacks'),
      path.join(os.homedir(), 'xpacks')
    );
  }
  return roots;
}

function findContentBins(root, depth) {
  if (depth < 0) return [];
  let entries;
  try {
    entries = fs.readdirSync(root, { withFileTypes: true });
  } catch {
    return [];
  }
  const bins = [];
  for (const entry of entries) {
    if (!entry.isDirectory() || entry.name === 'node_modules') continue;
    const dir = path.join(root, entry.name);
    if (entry.name === '.content') {
      const bin = path.join(dir, 'bin');
      if (fs.existsSync(bin)) bins.push(bin);
      continue;
    }
    bins.push(...findContentBins(dir, depth - 1));
  }
  return bins;
}

function works(exe) {
  const result = spawnSync(exe, ['--version'], { stdio: 'ignore' });
  return result.status === 0;
}

const binDirs = [];
for (const root of xpackRoots()) binDirs.push(...findContentBins(root, 6));

function resolveNinja() {
  if (process.env.CMAKE_MAKE_PROGRAM) {
    if (works(process.env.CMAKE_MAKE_PROGRAM)) return process.env.CMAKE_MAKE_PROGRAM;
  }
  for (const dir of binDirs) {
    const candidate = path.join(dir, exeName);
    if (fs.existsSync(candidate) && works(candidate)) return candidate;
  }
  for (const dir of (process.env.PATH || '').split(path.delimiter)) {
    if (!dir) continue;
    const candidate = path.join(dir, exeName);
    if (fs.existsSync(candidate) && works(candidate)) return candidate;
  }
  return null;
}

function dropStaleCache(buildDir) {
  const cacheFile = path.join(buildDir, 'CMakeCache.txt');
  let cache;
  try {
    cache = fs.readFileSync(cacheFile, 'utf8');
  } catch {
    return;
  }
  // кэш, собранный до фикса, содержит пути на обёртки из xpacks/.bin
  if (!/xpacks[\\/]\.bin/i.test(cache)) return;
  fs.rmSync(cacheFile, { force: true });
  fs.rmSync(path.join(buildDir, 'CMakeFiles'), { recursive: true, force: true });
  console.log('Сброшен устаревший кэш CMake (пути на xpacks/.bin)');
}

const ninja = resolveNinja();
if (!ninja) {
  console.error('Не найден ninja. Установите Ninja или выполните npm install.');
  process.exit(1);
}

const makeProgram = isWindows
  ? `-DCMAKE_MAKE_PROGRAM="${ninja.replace(/\\/g, '/')}"`
  : `-DCMAKE_MAKE_PROGRAM=${ninja}`;

// Тип сборки можно передать первым аргументом: `node scripts/configure.cjs Debug`.
// Для Debug используется отдельная папка build-debug, чтобы не ломать Release.
const extra = process.argv.slice(2);
const typeNames = ['debug', 'release', 'relwithdebinfo', 'minsizerel'];
const requested = extra.find((arg) => typeNames.includes(arg.toLowerCase())) || 'Release';
const rest = extra.filter((arg) => !typeNames.includes(arg.toLowerCase()));
const buildType = requested[0].toUpperCase() + requested.slice(1).toLowerCase();
const buildDir = buildType === 'Release' ? 'build' : `build-${buildType.toLowerCase()}`;

const args = [
  '-S', '.',
  '-B', buildDir,
  '-G', 'Ninja',
  `-DCMAKE_BUILD_TYPE=${buildType}`,
  makeProgram
].concat(rest);

const env = { ...process.env };
if (binDirs.length > 0) {
  env.PATH = binDirs.concat(env.PATH || '').join(path.delimiter);
}

console.log(`ninja: ${ninja}`);
console.log(`конфигурация: ${buildType} -> ${buildDir}`);
dropStaleCache(path.resolve(buildDir));

const result = spawnSync('cmake', args, {
  stdio: 'inherit',
  shell: isWindows,
  env
});

if (result.error) {
  console.error(result.error.message);
  process.exit(1);
}
process.exit(result.status === null ? 1 : result.status);
