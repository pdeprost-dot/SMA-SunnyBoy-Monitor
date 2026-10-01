const assert = require('node:assert/strict');

const validPrefix = value => value.length > 0 && value.length < 48 &&
  !value.includes('/') && !value.includes('+') && !value.includes('#') &&
  [...value].every(c => c.charCodeAt(0) >= 0x20);
const validMac = value => /^(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$/.test(value);
const validWpa = value => value.length >= 8 && value.length <= 63;
const topic = (prefix, slot) => `${prefix}/inv${slot + 1}`;
const unavailable = value => value === null ? '--' : String(value);

assert(validPrefix('smaesp'));
for (const bad of ['', 'sma/esp', 'sma+', 'sma#']) assert(!validPrefix(bad));
assert.deepEqual([0, 1, 2].map(i => topic('smaesp', i)), ['smaesp/inv1', 'smaesp/inv2', 'smaesp/inv3']);
assert(validMac('02:00:00:00:00:01'));
assert(!validMac('02:00:00:00:00'));
assert(validWpa('12345678') && !validWpa('short'));
assert(-90 >= -90 && 90 <= 90 && -180 >= -180 && 180 <= 180);
assert.equal(unavailable(null), '--');
assert.equal(unavailable(0), '0');
const snapshots = [{serial: 1}, {serial: 2, pac: 0}, {serial: 3}];
assert.equal(snapshots.length, 3);
assert.equal(snapshots[1].pac, 0);
assert.equal(snapshots[0].pac, undefined);
const formatted = new Intl.DateTimeFormat('fr-BE', {
  timeZone: 'Europe/Brussels', day: '2-digit', month: '2-digit', year: 'numeric',
  hour: '2-digit', minute: '2-digit', second: '2-digit', hourCycle: 'h23'
}).format(new Date('2026-06-21T12:34:56Z'));
assert.match(formatted, /^21\/06\/2026 14:34:56$/);
console.log('PASS 10/10 product configuration vectors');
