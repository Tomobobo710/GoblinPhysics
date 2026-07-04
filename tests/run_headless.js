/**
 * Headless entry point for the Goblin test suite. Loads the runner + every test file, runs all tests
 * (Chandler's Suite first, then Tom's) by stepping the physics world / calling geometry queries and
 * inspecting results, and prints a grouped pass/fail report. Dependency-free — just node.
 *
 *   node tests/run_headless.js               run everything
 *   node tests/run_headless.js gravity       run only groups whose name contains "gravity"
 *   node tests/run_headless.js --suite=tom   run only one suite
 */
var fs = require('fs');
var path = require('path');
var Runner = require('./js/runner.js');

// Load every test file in each suite folder (chandler first, then tom), in filename order.
['chandler', 'tom'].forEach(function (suiteDir) {
	var dir = path.join(__dirname, 'js', suiteDir);
	if (!fs.existsSync(dir)) return;
	fs.readdirSync(dir).filter(function (f) { return f.endsWith('.js'); }).sort().forEach(function (f) {
		require(path.join(dir, f));
	});
});

var arg = process.argv[2] || null;
var onlySuite = null, only = null;
if (arg && arg.indexOf('--suite=') === 0) onlySuite = arg.slice(8);
else only = arg;
var filter = function (t) {
	if (onlySuite && t.suite !== onlySuite) return false;
	if (only && t.group.indexOf(only) === -1) return false;
	return true;
};

console.log('=== Goblin test suite (headless) ===');

var SUITE_NAMES = { chandler: "Chandler's Suite", tom: "Tom's Suite" };
var curSuite = null, curGroup = null;
var summary = Runner.run(filter, function (r) {
	if (r.suite !== curSuite) { curSuite = r.suite; curGroup = null; console.log('\n##### ' + (SUITE_NAMES[curSuite] || curSuite) + ' #####'); }
	if (r.group !== curGroup) { curGroup = r.group; console.log('  [' + curGroup + ']'); }
	console.log('    ' + (r.ok ? 'ok  ' : 'FAIL') + '  ' + r.name + (r.ok ? '' : '   -> ' + r.error));
});

console.log('\n=== ' + summary.pass + ' passed, ' + summary.fail + ' failed (' + summary.total + ' total) ===');
process.exit(summary.fail > 0 ? 1 : 0);
