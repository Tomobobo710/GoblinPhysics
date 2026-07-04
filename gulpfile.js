const { src, dest, series, parallel } = require('gulp');
const uglify = require('gulp-uglify');
const concat = require('gulp-concat');
const jshint = require('gulp-jshint');
const yuidoc = require('yuidocjs');
const pkg = require('./package.json');

function lint() {
    return src('src/classes/**/*.js')
        .pipe(jshint())
        .pipe(jshint.reporter('default'));
}

const buildOrder = [
    'src/intro.js',
    'src/classes/Math/**.js',
    'src/libglobals.js',
    'src/classes/EventEmitter.js',
    'src/classes/RigidBody.js',
    'src/classes/ForceGenerator.js',
    'src/classes/BroadPhases/BasicBroadphase.js',
    'src/classes/BroadPhases/SAPBroadphase.js',
    'src/classes/BroadPhases/**/*.js',
    'src/classes/Collision/**/*.js',
    'src/classes/Constraints/**/*.js',
    'src/classes/ForceGenerators/**/*.js',
    'src/classes/RayTracing/**/*.js',
    // Nested Shapes subdirs listed explicitly BEFORE the broad Shapes glob: gulp's src() streams
    // deep-glob files (e.g. Shapes/Swept/*.js) out of order and can emit them AFTER outro.js, which
    // leaves that shape defined outside the module wrapper (breaks CommonJS require of the build).
    // Pinning them here (first match wins its position under gulp's de-dup) keeps them inside.
    'src/classes/Shapes/Swept/**/*.js',
    'src/classes/Shapes/**/*.js',
    'src/classes/Utils/**/*.js',
    'src/classes/*.js',
    'src/outro.js'
];

function build() {
    return src(buildOrder)
        .pipe(concat('goblin.js'))
        .pipe(dest('build'));
}

function buildMinified() {
    return src(buildOrder)
        .pipe(concat('goblin.min.js'))
        .pipe(uglify())
        .pipe(dest('build'));
}

// Compiles the @class/@method/@param YUIDoc comment blocks in src/ into a browsable API site
// under docs/. Ported from this project's original Gruntfile (grunt-contrib-yuidoc, paths:
// 'src', outdir: 'docs') to the yuidocjs library it wrapped, run directly under gulp.
function docs(done) {
    const options = yuidoc.Options(['src', '--outdir', 'docs']);
    options.project = {
        name: pkg.name,
        description: pkg.description,
        version: pkg.version,
        url: pkg.homepage
    };

    const json = new yuidoc.YUIDoc(options).run();
    new yuidoc.DocBuilder(options, json).compile(done);
}

exports.default = series(lint, parallel(build, buildMinified));
exports.docs = docs;