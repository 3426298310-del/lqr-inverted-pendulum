%VERIFY Run from the repository root after cloning.
run_reproduction;
testResults=runtests('tests');
assertSuccess(testResults);
fprintf('PASS: %d tests; reproduction outputs regenerated.\n',numel(testResults));
