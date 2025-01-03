const { src, dest, series, parallel } = require('gulp');
const uglify = require('gulp-uglify');
const concat = require('gulp-concat');
const jshint = require('gulp-jshint');

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

exports.default = series(lint, parallel(build, buildMinified));