import Vue from 'vue'
import App from './App.vue'
import router from './router'
import store from './store'
import VueApollo from 'vue-apollo'
import { ApolloClient } from 'apollo-client'
import { createHttpLink } from 'apollo-link-http'
import { InMemoryCache } from 'apollo-cache-inmemory'
import { provide } from '@vue/composition-api'
import { DefaultApolloClient } from '@vue/apollo-composable'
import { setContext } from '@apollo/client/link/context';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { library } from '@fortawesome/fontawesome-svg-core'
import { faUser, faArrowLeft, faPlus, faLock, faChevronDown, faChevronUp } from '@fortawesome/free-solid-svg-icons'

library.add(faUser, faArrowLeft, faPlus, faLock, faChevronDown, faChevronUp)

Vue.component('font-awesome-icon', FontAwesomeIcon)
Vue.config.productionTip = false

const httpLink = createHttpLink({
  uri: 'http://localhost:4000/graphql',
})

const authLink = setContext((_, { headers }) => {
  // get the authentication token from local storage if it exists
  const token = localStorage.getItem('token');
  // return the headers to the context so httpLink can read them
  return {
    headers: {
      ...headers,
      authorization: token ? `Bearer ${token}` : "",
    }
  }
});

const cache = new InMemoryCache()

const apolloClient = new ApolloClient({
  link: authLink.concat(httpLink),
  cache,
})

Vue.use(VueApollo)
Vue.prototype.$log = console.log

const apolloProvider = new VueApollo({
  defaultClient: apolloClient,
})


new Vue({
  setup () {
    provide(DefaultApolloClient, apolloClient)
  },
  apolloProvider,
  router,
  store,
  render: h => h(App)
}).$mount('#app')
