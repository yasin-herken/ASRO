import './App.css';
import React  from 'react';

import NavigatioBar from './components/NavigationBar/NavigatioBar';
import Interface from './components/Interface/Interface';

function App() {
	return (
		<div className="App">
			<NavigatioBar />
			<hr width="100%" color="#e5e5e5" />
			<Interface />
		</div>
	);
}

export default App;
