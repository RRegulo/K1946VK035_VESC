const https = require('https');
const fs = require('fs');
const path = require('path');
const os = require('os');
const crypto = require('crypto');
const tar = require('tar');
const unzipper = require('unzipper');
const { HttpsProxyAgent } = require('https-proxy-agent');

const releases = {
  'linux-x64': ['linux-x64.tar.gz', 'e7aa88b3501b02410df07892c35377b41e628cd9db876205914660b6becbbb9f'],
  'win32-x64': ['win32-x64.zip', 'a84f7167bf68b76d42cc3dc3c052efec63bc6aa630ebd35cdbc8ad082e4518d0']
};
const release = releases[`${process.platform}-${process.arch}`];
if (!release) {
  console.error('OpenOCD для этой платформы нужно установить вручную.');
  process.exit(1);
}
const [suffix, checksum] = release;
const archiveName = `xpack-openocd-k1921vk-0.12.0-k1921vk-${suffix}`;
const url = `https://github.com/DCVostok/openocd-k1921vk/releases/download/v0.12.0-k1921vk/${archiveName}`;
const destination = path.resolve('tools/openocd');
const exe = path.join(destination, 'bin', process.platform === 'win32' ? 'openocd.exe' : 'openocd');
const targetCfg = path.join(destination, 'openocd', 'scripts', 'target', 'k1921vk035.cfg');
const bundledTargetCfg = path.join(__dirname, '..', '.vscode', 'k1921vk035.cfg');
const proxy = process.env.HTTPS_PROXY || process.env.https_proxy;
const agent = proxy ? new HttpsProxyAgent(proxy) : undefined;

function download(address, file, redirects = 0) {
  return new Promise((resolve, reject) => {
    if (redirects > 8) return reject(new Error('Слишком много перенаправлений'));
    https.get(address, { agent }, response => {
      if (response.statusCode >= 300 && response.statusCode < 400 && response.headers.location) {
        response.resume();
        return resolve(download(new URL(response.headers.location, address), file, redirects + 1));
      }
      if (response.statusCode !== 200) {
        response.resume();
        return reject(new Error(`HTTP ${response.statusCode}: ${address}`));
      }
      const output = fs.createWriteStream(file);
      response.pipe(output);
      output.on('finish', () => output.close(resolve));
      output.on('error', reject);
      response.on('error', reject);
    }).on('error', reject);
  });
}

function installTargetConfig() {
  if (fs.existsSync(targetCfg)) return;
  fs.mkdirSync(path.dirname(targetCfg), { recursive: true });
  fs.copyFileSync(bundledTargetCfg, targetCfg);
}

async function main() {
  if (fs.existsSync(exe)) {
    installTargetConfig();
    return console.log('OpenOCD уже установлен:', exe);
  }
  const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'vk035-openocd-'));
  try {
    const archive = path.join(temporary, archiveName);
    console.log('Загрузка OpenOCD для К1921ВК035...');
    await download(url, archive);
    const actual = crypto.createHash('sha256').update(fs.readFileSync(archive)).digest('hex');
    if (actual !== checksum) throw new Error('Контрольная сумма архива OpenOCD не совпадает');
    fs.mkdirSync(destination, { recursive: true });
    if (suffix.endsWith('.zip')) {
      await fs.createReadStream(archive).pipe(unzipper.Extract({ path: temporary })).promise();
      fs.cpSync(path.join(temporary, 'xpack-openocd-k1921vk-0.12.0-k1921vk'), destination, { recursive: true });
    } else {
      await tar.x({ file: archive, cwd: destination, strip: 1 });
    }
    installTargetConfig();
    if (!fs.existsSync(exe) || !fs.existsSync(targetCfg)) throw new Error('OpenOCD не найден в распакованном архиве');
    console.log('OpenOCD установлен:', exe);
  } finally {
    fs.rmSync(temporary, { recursive: true, force: true });
  }
}
main().catch(error => { console.error(error.message); process.exitCode = 1; });
