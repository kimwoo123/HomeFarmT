pipeline {
	agent none
	tools {nodejs 'nodejs'}
	stages {
		stage('Frontend Build') {
			agent any
			steps {
				dir ('frontend') {
					sh 'cp /security/frontend/.env ./.env'
					sh 'npm install --legacy-peer-deps .'
					sh 'npm run build'
				}
				dir ('backend') {
					sh 'cp /security/backend/.env ./.env'
				}
			}
		}
		stage('Backend Build') {
			agent any
			steps {
				sh 'docker-compose down --rmi all'
				sh 'docker-compose up -d'
			}
		}
	}
}
