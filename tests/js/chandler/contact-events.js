// Chandler's contact-events.html — ported.
// A static sphere and a dynamic sphere fired straight down at -10, gravity off. The narrowphase fires
// lifecycle events on the bodies: speculativeContact (predicted, and preventable by returning false),
// contact (touching), endContact (separated). His test: the first speculativeContact is prevented, the
// real contact fires with exactly 2 speculative contacts seen, and an endContact fires on separation.
//
// Live: we listen for the events as the ball falls in and bounces out, and each criterion flips green
// the tick its event actually fires — you watch the contact land and then release.
(function (Runner) {
	Runner.suite('chandler');

	Runner.test('contact-events', 'speculative prevention + contact + endContact fire correctly', function (t) {
		t.log('The engine emits contact lifecycle events; we listen as the ball hits and leaves.');

		var w = t.makeWorld({ gravity: 0 });
		var s1 = t.sphere(w, 1, 0, { color: '#888' });                                    // static
		var s2 = t.sphere(w, 1, 1, { pos: [0, 5, 0], vel: [0, -10, 0], color: '#45B7D1' }); // dropped at -10

		var seenSpeculative = 0, hasPreventedOne = false, firstContact = null, endContactFired = false;
		s1.addListener('speculativeContact', function () {
			seenSpeculative += 1;
			if (hasPreventedOne === false) { hasPreventedOne = true; return false; } // prevent the first
		});
		s1.addListener('contact', function () { if (firstContact === null) firstContact = { prevented: hasPreventedOne, seen: seenSpeculative }; });
		s1.addListener('endContact', function () { endContactFired = true; });

		// Each criterion becomes true the tick its event has fired with the right state.
		t.expect('first predicted contact is prevented, then contact fires after 2 speculative contacts', function () {
			return { ok: firstContact !== null && firstContact.prevented === true && firstContact.seen === 2,
				detail: firstContact ? ('contact seen after ' + firstContact.seen + ' speculative') : 'waiting for contact' };
		});
		t.expect('an endContact fires as the ball separates', function () {
			return { ok: endContactFired, detail: endContactFired ? 'endContact fired' : 'waiting for separation' };
		});

		t.log('Ball falls in (contact), bounces, and leaves (endContact) — checks flip as events fire.');
		t.simulate(w, 120);
	}, {
		visual: true, steps: 120, page: 'contact-events',
		description:
			"The engine reports contact lifecycle events on a body: a predicted 'speculativeContact' (which " +
			"a listener can veto by returning false), a real 'contact' when surfaces touch, and an " +
			"'endContact' when they separate. A ball is fired down at a fixed sphere: the first predicted " +
			"contact is vetoed, the real contact then fires after exactly two speculative contacts, and as " +
			"the ball bounces away an endContact fires. Each check flips green when its event actually " +
			"happens. PASS: the prevention, the contact, and the endContact all occur correctly."
	});

})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.GoblinRunner);
