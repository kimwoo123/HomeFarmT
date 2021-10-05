module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  email: { type: DataTypes.STRING, unique: true },
  userid: { type: DataTypes.INTEGER, autoIncrement: true, primaryKey: true },
  hashid: { type: DataTypes.STRING },
  password: { type: DataTypes.STRING, defaultValue: 'none' },
  route1: { type: DataTypes.STRING },
  route2: { type: DataTypes.STRING },
  route3: { type: DataTypes.STRING },
  region: { type: DataTypes.STRING },
  turtlebot: { type: DataTypes.STRING },
}, {
  timestamps: false
});
